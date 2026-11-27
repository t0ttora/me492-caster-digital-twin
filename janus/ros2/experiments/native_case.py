#!/usr/bin/env python3
"""Bounded JANUS scenarios using rendered Gazebo camera data and real ROS transport."""
import argparse
from collections import Counter
from contextlib import ExitStack
from dataclasses import fields
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import cv2
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, String

from arche_experiments.ros_fields import joint_fields, odometry_fields, stamp_s, twist_fields
from arche_experiments.vision import CenteringGate, Observation, unpack_image


NOMINAL_TAG = 'janus_dock'
TAG_SIDE_M = .24 * 21 / 29
TAG_X_M = 1.50
TAG_Z_M = .31
CAMERA_X_M = .25
STANDOFF_M = .265
POSITION_TOLERANCE_M = .02
FRAME_FIELDS = {field.name for field in fields(Observation)}


class NativeRun(Node):
    def __init__(self, case_id, run_dir, trace, service_log, camera_bridge_pid):
        super().__init__('janus_live_case')
        self.case_id = case_id
        self.run_dir = run_dir
        self.trace = trace
        self.service_log = service_log
        self.camera_bridge_pid = camera_bridge_pid
        self.bridge_paused = False
        self.clock_pause_wall = None
        self.trace_start_wall = time.monotonic()
        self.gate = CenteringGate()
        self.publish = self.create_publisher(Twist, '/janus/cmd_vel/request', 10)
        self.publish_abort = self.create_publisher(Bool, '/janus/abort', 10)
        self.publish_fault_image = None
        if camera_bridge_pid:
            self.publish_fault_image = self.create_publisher(Image, '/janus/camera/image_raw', qos_profile_sensor_data)
        self.now = None
        self.start = None
        self.world = None
        self.start_pose = None
        self.frame = None
        self.frame_focal = None
        self.latest_image = None
        self.guard = None
        self.pending_image = 'start'
        self.events = {}
        self.counts = Counter()
        self.service_processes = []
        for cls, topic, name in (
            (Clock, '/clock', 'clock'), (Odometry, '/janus/ground_truth', 'truth'),
            (Odometry, '/odom', 'odom'), (JointState, '/joint_states', 'joint_state'),
            (Image, '/janus/camera/image_raw', 'image'), (CameraInfo, '/janus/camera/camera_info', 'intrinsics'),
            (String, '/janus/qr/observation', 'qr'), (Twist, '/cmd_vel', 'guarded'),
            (String, '/janus/command_guard/status', 'status'),
        ):
            self.create_subscription(cls, topic, getattr(self, name), qos_profile_sensor_data)

    def write(self, source, payload):
        self.counts[source] += 1
        self.trace.write(json.dumps({'source': source, 'read_time_s': self.now,
                                     'read_wall_s': time.monotonic()-self.trace_start_wall, **payload},
                                    allow_nan=False) + '\n')

    def event(self, name, **details):
        self.events[name] = self.now
        self.write('event', {'event': name, **details})

    def clock(self, msg):
        tick = stamp_s(msg.clock)
        if self.now is None or int(self.now * 20 + 1e-9) != int(tick * 20 + 1e-9):
            self.write('clock', {'time_s': tick})
        self.now = tick

    def pose(self, msg, coordinates):
        try:
            return odometry_fields(msg, coordinates)
        except ValueError as exc:
            self.write(coordinates + '_error', {'error': str(exc), 'frame': msg.header.frame_id,
                                                 'child': msg.child_frame_id})
            return None

    def truth(self, msg):
        if pose := self.pose(msg, 'map'):
            self.world = pose
            self.write('truth', pose)

    def odom(self, msg):
        if pose := self.pose(msg, 'odom'):
            self.write('odom', pose)

    def joint_state(self, msg):
        try:
            row = joint_fields(msg)
        except ValueError as exc:
            self.write('joint_error', {'error': str(exc), 'names': list(msg.name)})
            return
        if self.counts['joint'] == 0 or int(row['time_s'] * 50) != self.previous_joint_slot:
            self.previous_joint_slot = int(row['time_s'] * 50)
            self.write('joint', row)

    def image(self, msg):
        if not self.bridge_paused:
            self.latest_image = msg
        if self.pending_image is None:
            return
        pixels = unpack_image(msg.data, msg.width, msg.height, msg.step, msg.encoding)
        image_file = self.run_dir / (self.pending_image + '-rendered.png')
        if cv2.imwrite(str(image_file), pixels):
            self.write('image', {'file': image_file.name, 'capture_time_s': stamp_s(msg.header.stamp)})
            self.pending_image = None

    def intrinsics(self, msg):
        if self.frame_focal is None and msg.k[0] > 0:
            self.frame_focal = msg.k[0]
            self.write('intrinsics', {'fx_px': msg.k[0], 'frame_id': msg.header.frame_id})

    def qr(self, msg):
        row = json.loads(msg.data)
        self.frame = Observation(**{key: value for key, value in row.items() if key in FRAME_FIELDS})
        row['observation_source'] = row.pop('source')
        self.write('qr', row)

    def guarded(self, msg):
        row = {'time_s': self.now, **twist_fields(msg)}
        self.write('guard_output', row)

    def status(self, msg):
        self.guard = json.loads(msg.data)
        self.write('guard_status', self.guard)

    def send(self, v, yaw, state):
        command = Twist()
        command.linear.x, command.angular.z = v, yaw
        self.publish.publish(command)
        error = self.frame.error if self.frame is not None else None
        row = {'time_s': self.now, 'v_mps': v, 'yaw_rate_radps': yaw, 'state': state,
               'capture_time_s': self.frame.capture_time_s if self.frame is not None else None,
               'normalized_error': error}
        self.write('request', row)

    def service(self, path, reqtype, request):
        process = subprocess.Popen(['gz', 'service', '-s', path, '--reqtype', reqtype,
                                    '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', request],
                                   stdout=self.service_log, stderr=subprocess.STDOUT)
        self.service_processes.append(process)

    def set_marker_y(self, y, event):
        self.event(event, marker_y_m=y)
        request = f'name: "{NOMINAL_TAG}" position: {{ x: {TAG_X_M} y: {y} z: {TAG_Z_M} }} orientation: {{ w: 1 }}'
        self.service('/world/janus/set_pose', 'gz.msgs.Pose', request)

    def change_camera_bridge(self, paused):
        if paused == self.bridge_paused:
            return
        cmdline = Path(f'/proc/{self.camera_bridge_pid}/cmdline').read_bytes()
        if b'parameter_bridge' not in cmdline or b'camera_bridge.yaml' not in cmdline:
            raise RuntimeError('Q13 requires the camera bridge PID from this isolated launch')
        os.kill(self.camera_bridge_pid, signal.SIGSTOP if paused else signal.SIGCONT)
        self.bridge_paused = paused

    def qr_command(self):
        visible = (self.frame is not None and self.frame.valid
                   and 0 <= self.now - self.frame.capture_time_s <= .20)
        forward = self.gate.update(self.now, self.frame)
        if not visible:
            if 'hide' in self.events and 'visible_loss' not in self.events:
                self.event('visible_loss')
                self.pending_image = 'loss'
            self.send(0., 0., 'qr_missing_invalid_or_expired')
            return
        turn = max(-.45, min(.45, -1.8 * self.frame.error))
        if not forward or self.frame_focal is None:
            self.send(0., turn, 'centering')
            return
        c0, c1, c2, c3 = self.frame.corners_px
        apparent_side = (math.dist(c0, c3) + math.dist(c1, c2)) * .5
        estimated_depth = self.frame_focal * TAG_SIDE_M / apparent_side
        if estimated_depth <= STANDOFF_M:
            if 'standoff' not in self.events:
                self.event('standoff', estimated_optical_depth_m=estimated_depth)
                self.pending_image = 'standoff'
            self.send(0., 0., 'standoff_hold')
        else:
            self.send(min(.12, max(.018, .8*(estimated_depth-STANDOFF_M))), turn, 'approach')

    def step(self):
        if self.now is None or self.world is None:
            return False
        if self.start is None:
            self.start = self.now
            self.start_pose = self.world
            self.event('case_start')
        elapsed = self.now - self.start
        if self.case_id in ('Q01', 'Q02', 'Q03', 'Q06', 'Q08', 'Q13'):
            if self.case_id == 'Q06':
                if 'hide' not in self.events and self.world['v_mps'] > .05:
                    self.set_marker_y(4., 'hide')
                if 'hide' in self.events and self.now-self.events['hide'] >= 1.0 and 'restore' not in self.events:
                    self.set_marker_y(0., 'restore')
            if self.case_id == 'Q13':
                if 'camera_cut' not in self.events and self.world['v_mps'] > .05 and self.latest_image is not None:
                    self.change_camera_bridge(True)
                    self.event('camera_cut', last_capture_s=stamp_s(self.latest_image.header.stamp))
                if 'camera_cut' in self.events and 'replay_old_frame' not in self.events:
                    if self.now-self.events['camera_cut'] >= .40:
                        self.publish_fault_image.publish(self.latest_image)
                        self.event('replay_old_frame', capture_s=stamp_s(self.latest_image.header.stamp))
                if 'camera_cut' in self.events and 'camera_restore' not in self.events:
                    if self.now-self.events['camera_cut'] >= 1.:
                        self.change_camera_bridge(False)
                        self.event('camera_restore')
            self.qr_command()
            if self.case_id == 'Q08':
                return elapsed > 2.5
            if self.case_id == 'Q13':
                return 'camera_restore' in self.events and self.now - self.events['camera_restore'] >= 1.2
            return 'standoff' in self.events and self.now - self.events['standoff'] >= .60
        elif self.case_id == 'M01':
            self.send(0., 0., 'zero_twist')
            return elapsed >= 2.
        elif self.case_id == 'M02':
            if elapsed < .50:
                self.send(0., 0., 'settling')
            elif elapsed < 2.50:
                self.send(.12, 0., 'forward')
            else:
                if 'brake' not in self.events:
                    self.event('brake')
                self.send(0., 0., 'brake')
            return elapsed >= 3.5
        elif self.case_id == 'O04':
            if elapsed < 1.20:
                self.send(.10, 0., 'forward')
            elif 'cut' not in self.events:
                self.event('cut')
            elif elapsed >= 2.65:
                if 'abort_injected' not in self.events:
                    self.publish_abort.publish(Bool(data=True))
                    self.event('abort_injected')
                self.send(.08, 0., 'deliberate_post_abort_probe')
            elif elapsed >= 1.95:
                self.send(.08, 0., 'explicit_second_request')
            return elapsed >= 3.75
        elif self.case_id == 'O08':
            if elapsed < 1.20:
                self.send(.10, 0., 'forward')
            elif 'clock_cut' not in self.events:
                self.event('clock_cut')
                self.service('/world/janus/control', 'gz.msgs.WorldControl', 'pause: true')
                self.clock_pause_wall = time.monotonic()
            elif 'clock_restore' not in self.events and time.monotonic()-self.clock_pause_wall >= 1.2:
                self.event('clock_restore')
                self.service('/world/janus/control', 'gz.msgs.WorldControl', 'pause: false')
            elif elapsed >= 2.30 and 'clock_restore' in self.events:
                if 'authorized_restart_input' not in self.events:
                    self.event('authorized_restart_input', authority='explicit_bounded_test_step')
                self.send(.08, 0., 'new_request')
            return elapsed >= 3.75
        raise ValueError(f'Case implementation missing: {self.case_id}')

    def finish(self):
        if self.bridge_paused:
            self.change_camera_bridge(False)
        if self.clock_pause_wall is not None and 'clock_restore' not in self.events:
            self.service('/world/janus/control', 'gz.msgs.WorldControl', 'pause: false')
        if self.now is not None:
            self.send(0., 0., 'end')
        for service in self.service_processes:
            try:
                if service.wait(timeout=4) != 0:
                    self.counts['fixture_service_error'] += 1
            except subprocess.TimeoutExpired:
                service.terminate()
                self.counts['fixture_service_error'] += 1


def parse():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case', choices=['M01', 'M02', 'Q01', 'Q02', 'Q03', 'Q06', 'Q08', 'Q13', 'O04', 'O08'])
    parser.add_argument('output', type=Path)
    parser.add_argument('--wall-s', type=float, default=75.)
    parser.add_argument('--camera-bridge-pid', type=int)
    args = parser.parse_args()
    if args.case == 'Q13' and (args.camera_bridge_pid is None or args.camera_bridge_pid <= 1):
        parser.error('Q13 needs the camera bridge PID from the current isolated launch')
    return args


def main():
    args = parse()
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    with ExitStack() as close:
        trace = close.enter_context((args.output/'telemetry.jsonl').open('w'))
        services = close.enter_context((args.output/'simulation-service.log').open('w'))
        node = NativeRun(args.case, args.output, trace, services, args.camera_bridge_pid)
        try:
            next_tick = time.monotonic()
            deadline = next_tick + args.wall_s
            ended = False
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.005)
                if time.monotonic() >= next_tick:
                    next_tick = time.monotonic() + .025
                    if node.step():
                        ended = True
                        break
            authority = {topic: node.count_publishers(topic) for topic in
                         ('/clock', '/janus/ground_truth', '/odom', '/cmd_vel')}
        finally:
            try:
                node.finish()
            finally:
                node.destroy_node()
                rclpy.try_shutdown()
        summary = {'case_id': args.case, 'ran_until_fixture_end': ended, 'clock_start_s': node.start,
                   'clock_end_s': node.now, 'events_s': node.events, 'samples': node.counts,
                   'publishers_per_runtime_topic': authority,
                   'final_world_pose': node.world, 'scope': 'SSD-hosted Linux VM / ROS 2 Jazzy / Gazebo Harmonic'}
        (args.output/'execution.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
        print(json.dumps(summary, indent=2, allow_nan=False))
    return 0 if ended else 2


if __name__ == '__main__':
    raise SystemExit(main())
