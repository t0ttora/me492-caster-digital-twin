#!/usr/bin/env python3
"""Bounded JANUS scenarios using rendered Gazebo camera data and real ROS transport."""
import argparse
from collections import Counter, deque
from contextlib import ExitStack
from copy import deepcopy
from dataclasses import fields
from hashlib import sha256
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
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
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
QR_CASES = ('Q01', 'Q02', 'Q03', 'Q04', 'Q05', 'Q06', 'Q07', 'Q08', 'Q10', 'Q13', 'Q17', 'Q19')


class NativeRun(Node):
    def __init__(self, case_id, run_dir, trace, service_log, camera_bridge_pid, q13_mode):
        super().__init__('janus_live_case')
        self.case_id = case_id
        self.run_dir = run_dir
        self.trace = trace
        self.service_log = service_log
        self.camera_bridge_pid = camera_bridge_pid
        self.q13_mode = q13_mode
        self.bridge_paused = False
        self.replay_last_try = None
        self.replayed_stamp = None
        self.replay_image = None
        self.recent_images = deque(maxlen=7)
        self.clock_pause_wall = None
        self.trace_start_wall = time.monotonic()
        self.search_direction = -1
        self.retry_losses = 0
        self.marker_hidden = False
        self.optical_mode = 'normal'
        self.photometric_samples = set()
        self.gate = CenteringGate()
        self.publish = self.create_publisher(Twist, '/janus/cmd_vel/request', 10)
        self.publish_abort = self.create_publisher(Bool, '/janus/abort', 10)
        self.publish_authorize = (self.create_publisher(Bool, '/janus/recovery/authorize', 10)
                                  if case_id == 'O08' else None)
        self.publish_fault_image = None
        if camera_bridge_pid or case_id == 'Q10':
            fault_qos = QoSProfile(depth=3, reliability=ReliabilityPolicy.RELIABLE) if camera_bridge_pid else qos_profile_sensor_data
            self.publish_fault_image = self.create_publisher(Image, '/janus/camera/image_raw', fault_qos)
        self.publish_fault_odom = (self.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
                                   if case_id == 'O05' else None)
        self.now = None
        self.start = None
        self.world = None
        self.start_pose = None
        self.frame = None
        self.frame_focal = None
        self.latest_image = None
        self.latest_odometry = None
        self.guard = None
        self.pending_image = 'start'
        self.episode_frames = []
        self.events = {}
        self.counts = Counter()
        self.service_processes = []
        latest_only = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        image_topic = '/janus/camera/native_raw' if case_id == 'Q10' else '/janus/camera/image_raw'
        for cls, topic, name in (
            (Clock, '/clock', 'clock'), (Odometry, '/janus/ground_truth', 'truth'),
            (Odometry, '/odom', 'odom'), (JointState, '/joint_states', 'joint_state'),
            (Image, image_topic, 'image'), (CameraInfo, '/janus/camera/camera_info', 'intrinsics'),
            (String, '/janus/qr/observation', 'qr'), (Twist, '/cmd_vel', 'guarded'),
            (String, '/janus/command_guard/status', 'status'),
        ):
            qos = latest_only if cls in (Clock, JointState, Image) else qos_profile_sensor_data
            self.create_subscription(cls, topic, getattr(self, name), qos)

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
            self.latest_odometry = msg
            self.write('odom', pose)

    def joint_state(self, msg):
        slot = int(stamp_s(msg.header.stamp) * 50)
        if self.counts['joint'] and slot == self.previous_joint_slot:
            return
        try:
            row = joint_fields(msg)
        except ValueError as exc:
            self.write('joint_error', {'error': str(exc), 'names': list(msg.name)})
            return
        self.previous_joint_slot = slot
        self.write('joint', row)

    def image(self, msg):
        if not self.bridge_paused:
            self.latest_image = msg
            if self.case_id == 'Q13':
                self.recent_images.append(msg)
        if self.case_id == 'Q10':
            self.relay_photometric_fixture(msg)
        capture = stamp_s(msg.header.stamp)
        record_episode = (
            self.case_id in ('Q01', 'Q02', 'Q03', 'Q04', 'Q05', 'Q06', 'Q07', 'Q19') and self.world is not None
            and self.world['v_mps'] > .02 and len(self.episode_frames) < 36
            and (not self.episode_frames or capture - self.episode_frames[-1]['capture_time_s'] >= .24)
        )
        if self.pending_image is None and not record_episode:
            return
        pixels = unpack_image(msg.data, msg.width, msg.height, msg.step, msg.encoding)
        if self.pending_image is not None:
            image_file = self.run_dir / (self.pending_image + '-rendered.png')
            if cv2.imwrite(str(image_file), pixels):
                self.write('image', {'file': image_file.name, 'capture_time_s': capture})
                self.pending_image = None
        if record_episode:
            directory = self.run_dir / 'episode-frames'
            directory.mkdir(exist_ok=True)
            image_file = directory / f'{len(self.episode_frames)+1:03}.png'
            if cv2.imwrite(str(image_file), pixels):
                self.episode_frames.append({'capture_time_s': capture, 'file': str(image_file.relative_to(self.run_dir)),
                    'width_px': msg.width, 'height_px': msg.height, 'encoding': msg.encoding,
                    'sha256': sha256(image_file.read_bytes()).hexdigest()})

    def relay_photometric_fixture(self, msg):
        if self.optical_mode == 'normal':
            self.publish_fault_image.publish(msg)
            return
        original = unpack_image(msg.data, msg.width, msg.height, msg.step, msg.encoding)
        adjusted = original.copy()
        cv2.ellipse(adjusted, (msg.width//2, msg.height//2), (86, 76), 0, 0, 360,
                    (255,255,255), thickness=-1)
        forwarded = Image(header=msg.header, height=msg.height, width=msg.width,
                          encoding='bgr8', is_bigendian=0, step=3*msg.width, data=adjusted.tobytes())
        self.publish_fault_image.publish(forwarded)
        if self.optical_mode not in self.photometric_samples:
            self.photometric_samples.add(self.optical_mode)
            source, modified = (self.run_dir/f'{self.optical_mode}-{label}.png' for label in ('source-rendered','derived'))
            cv2.imwrite(str(source), original)
            cv2.imwrite(str(modified), adjusted)
            self.write('photometric_injection', {'mode':self.optical_mode, 'capture_time_s':stamp_s(msg.header.stamp),
                       'source_file':source.name, 'derived_file': modified.name})

    def intrinsics(self, msg):
        if self.frame_focal is None and msg.k[0] > 0:
            self.frame_focal = msg.k[0]
            self.write('intrinsics', {'fx_px': msg.k[0], 'frame_id': msg.header.frame_id})

    def qr(self, msg):
        row = json.loads(msg.data)
        self.frame = Observation(**{key: value for key, value in row.items() if key in FRAME_FIELDS})
        row['observation_source'] = row.pop('source')
        self.write('qr', row)
        if (self.case_id == 'Q13' and self.replayed_stamp == row['capture_time_s'] and not row['valid'] and
                'old_frame_rejected' not in self.events):
            self.event('old_frame_rejected', retained_capture_s=self.replayed_stamp, reason=row['reason'],
                       capture_age_at_observer_s=row['arrival_time_s']-row['capture_time_s'])

    def guarded(self, msg):
        row = {'time_s': self.now, **twist_fields(msg)}
        self.write('guard_output', row)

    def status(self, msg):
        self.guard = json.loads(msg.data)
        self.write('guard_status', self.guard)
        if self.guard['reason'] == 'recovery_authorized':
            self.event('recovery_ack_received', source_guard_time_s=self.guard['time_s'])

    def send(self, v, yaw, state):
        command = Twist()
        command.linear.x, command.angular.z = v, yaw
        self.publish.publish(command)
        error = self.frame.error if self.frame is not None else None
        row = {'time_s': self.now, 'v_mps': v, 'yaw_rate_radps': yaw, 'state': state,
               'capture_time_s': self.frame.capture_time_s if self.frame is not None else None,
               'normalized_error': error}
        self.write('request', row)

    def service(self, path, reqtype, request, label):
        process = subprocess.Popen(['gz', 'service', '-s', path, '--reqtype', reqtype,
                                    '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', request],
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.service_processes.append((process,label))

    def read_services(self):
        waiting = []
        for process, label in self.service_processes:
            if process.poll() is None:
                waiting.append((process,label))
                continue
            stdout,_ = process.communicate()
            self.service_log.write(f'{label}: return code {process.returncode}\n{stdout}')
            if process.returncode == 0 and 'data: true' in stdout:
                self.event(f'{label}_service_ack')
            else:
                self.counts['fixture_service_error'] += 1
        self.service_processes = waiting

    def set_marker_y(self, y, event):
        self.event(event, marker_y_m=y)
        request = f'name: "{NOMINAL_TAG}" position: {{ x: {TAG_X_M} y: {y} z: {TAG_Z_M} }} orientation: {{ w: 1 }}'
        self.service('/world/janus/set_pose', 'gz.msgs.Pose', request, event)

    def change_camera_bridge(self, paused):
        if paused == self.bridge_paused:
            return
        proc = Path(f'/proc/{self.camera_bridge_pid}')
        cmdline = proc.joinpath('cmdline').read_bytes().split(b'\0')
        environment = proc.joinpath('environ').read_bytes().split(b'\0')
        domain = f"ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']}".encode()
        if (not cmdline or not cmdline[0].endswith(b'/parameter_bridge')
                or b'__node:=janus_camera_bridge' not in cmdline or domain not in environment):
            raise RuntimeError('Q13 requires the camera bridge PID from this isolated launch')
        os.kill(self.camera_bridge_pid, signal.SIGSTOP if paused else signal.SIGCONT)
        self.bridge_paused = paused

    def drive_q13_fault_sequence(self):
        if 'camera_cut' not in self.events and self.world['v_mps'] > .045 and self.latest_image is not None:
            if self.q13_mode == 'regression':
                if self.frame is None or not self.frame.valid:
                    return
                retained = [msg for msg in self.recent_images
                            if .02 <= self.frame.capture_time_s-stamp_s(msg.header.stamp) <= .10
                            and 0 <= self.now-stamp_s(msg.header.stamp) <= .135]
                if not retained:
                    return
                self.replay_image = max(retained,key=lambda msg: stamp_s(msg.header.stamp))
            else:
                self.replay_image = self.latest_image
            self.change_camera_bridge(True)
            capture = stamp_s(self.replay_image.header.stamp)
            self.event('camera_cut', retained_capture_s=capture,
                       previous_observed_capture_s=self.frame.capture_time_s if self.frame else None)
        if not self.bridge_paused:
            return
        elapsed = self.now-self.events['camera_cut']
        replay_stamp = stamp_s(self.replay_image.header.stamp)
        should_send = self.now-replay_stamp <= .196 if self.q13_mode == 'regression' else elapsed >= .40
        resend_s = .025 if self.q13_mode == 'regression' else .12
        if (should_send and 'old_frame_rejected' not in self.events and self.publish_fault_image.get_subscription_count()>0
                and (self.replay_last_try is None or self.now-self.replay_last_try >= resend_s)):
            self.replayed_stamp = replay_stamp
            self.replay_last_try = self.now
            self.publish_fault_image.publish(self.replay_image)
            if 'replay_old_frame' not in self.events:
                self.event('replay_old_frame', capture_s=self.replayed_stamp,
                           receiving_subscribers=self.publish_fault_image.get_subscription_count(),
                           pixel_payload_sha256=sha256(bytes(self.replay_image.data)).hexdigest())
        if elapsed >= 1.40:
            self.change_camera_bridge(False)
            self.event('camera_restore')

    def qr_command(self):
        if 'standoff' in self.events:
            self.send(0., 0., 'standoff_hold')
            return
        visible = (self.frame is not None and self.frame.valid
                   and 0 <= self.now - self.frame.capture_time_s <= .20)
        forward = self.gate.update(self.now, self.frame)
        if not visible:
            if 'hide' in self.events and 'visible_loss' not in self.events:
                self.event('visible_loss')
                self.pending_image = 'loss'
            if (self.case_id == 'Q05' and 'qr_acquired' not in self.events and
                    self.now-self.start >= .25 and self.now-self.start < 10.):
                if self.search_direction == -1 and 'search_started' not in self.events:
                    self.event('search_started', max_relative_angle_rad=math.radians(55))
                offset = math.remainder(self.world['yaw_rad'] - self.start_pose['yaw_rad'], 2*math.pi)
                if offset <= -math.radians(54) and self.search_direction == -1:
                    self.search_direction = 1
                rate = .4*self.search_direction
                if offset >= math.radians(54) and self.search_direction == 1:
                    rate = 0.
                self.send(0., rate, 'bounded_search')
            else:
                self.send(0., 0., 'qr_missing_invalid_or_expired')
            return
        if self.case_id == 'Q05' and 'qr_acquired' not in self.events:
            self.event('qr_acquired', station_id=self.frame.id)
        # Inside the acquisition band, hold the settled view through the dwell.
        # A tiny yaw correction on each raster quantization jump can prevent that dwell.
        turn = (0. if abs(self.frame.error) <= .005 or (not forward and abs(self.frame.error) <= .02)
                else max(-.45, min(.45, -1.8 * self.frame.error)))
        if not forward or self.frame_focal is None:
            self.send(0., turn, 'centering')
            return
        c0, c1, c2, c3 = self.frame.corners_px
        apparent_side = (math.dist(c0, c3) + math.dist(c1, c2)) * .5
        estimated_depth = self.frame_focal * TAG_SIDE_M / apparent_side
        if estimated_depth <= STANDOFF_M:
            if abs(self.frame.error) > .02:
                self.send(0., turn, 'centering_at_range')
                return
            self.event('standoff', estimated_optical_depth_m=estimated_depth)
            self.pending_image = 'standoff'
            self.send(0., 0., 'standoff_hold')
        else:
            self.send(min(.12, max(.018, .8*(estimated_depth-STANDOFF_M))), turn, 'approach')

    def step(self):
        if self.now is None or self.world is None:
            return False
        self.read_services()
        if self.start is None:
            self.start = self.now
            self.start_pose = self.world
            self.event('case_start')
        elapsed = self.now - self.start
        if self.case_id in QR_CASES and elapsed >= 1.05 and 'steady_sample_requested' not in self.events:
            self.event('steady_sample_requested')
            self.pending_image = 'steady'
        if self.case_id in QR_CASES:
            if self.case_id in ('Q06', 'Q07'):
                if 'hide' not in self.events and self.world['v_mps'] > .05:
                    self.set_marker_y(4., 'hide')
                outage = .80 if self.case_id == 'Q07' else 1.0
                if 'hide' in self.events and self.now-self.events['hide'] >= outage and 'restore' not in self.events:
                    self.set_marker_y(.12 if self.case_id == 'Q07' else 0., 'restore')
            if self.case_id == 'Q17':
                if (not self.marker_hidden and self.retry_losses <= 2 and self.world['v_mps'] > .05 and
                        self.now >= self.events.get(f'restore_{self.retry_losses}',-1)+.48):
                    self.retry_losses += 1
                    self.marker_hidden = True
                    self.set_marker_y(4., f'hide_{self.retry_losses}')
                if self.marker_hidden and f'hide_{self.retry_losses}' in self.events:
                    if self.now-self.events[f'hide_{self.retry_losses}'] >= .85:
                        if self.retry_losses <= 2:
                            self.set_marker_y(0., f'restore_{self.retry_losses}')
                            self.marker_hidden = False
                        elif 'retry_budget_exhausted' not in self.events:
                            self.event('retry_budget_exhausted', retries=2, reason='three_observed_loss_intervals')
                            self.publish_abort.publish(Bool(data=True))
                if 'retry_budget_exhausted' in self.events:
                    self.send(0., 0., 'retry_budget_exhausted')
                    return self.now-self.events['retry_budget_exhausted'] >= .60
            if self.case_id == 'Q13':
                self.drive_q13_fault_sequence()
            if self.case_id == 'Q10':
                if 'optical_exposure_start' not in self.events and self.world['v_mps'] > .04:
                    self.optical_mode = 'glare'
                    self.event('optical_exposure_start', mode=self.optical_mode)
            self.qr_command()
            if self.case_id == 'Q08':
                return elapsed > 2.5
            if self.case_id == 'Q13':
                if 'camera_restore' in self.events:
                    recovered_after = self.now-self.events['camera_restore']
                    if self.q13_mode != 'resume':
                        return recovered_after >= .15
                    if self.world['v_mps'] > .02 and recovered_after >= .45 and 'observed_restart_motion' not in self.events:
                        self.event('observed_restart_motion')
                    return ('observed_restart_motion' in self.events and
                            self.now-self.events['observed_restart_motion'] >= .50)
                return False
            if self.case_id == 'Q10':
                return 'optical_exposure_start' in self.events and self.now-self.events['optical_exposure_start'] >= 1.2
            if self.case_id in ('Q06','Q07') and 'restore' in self.events:
                return self.now-self.events['restore'] >= 2.7
            return ('standoff' in self.events and self.now-self.events['standoff'] >= .60
                    and self.frame is not None and self.frame.valid and abs(self.frame.error) <= .02
                    and 0 <= self.now-self.frame.capture_time_s <= .20)
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
                self.service('/world/janus/control', 'gz.msgs.WorldControl', 'pause: true', 'clock_cut')
                self.clock_pause_wall = time.monotonic()
            elif 'clock_restore' not in self.events and time.monotonic()-self.clock_pause_wall >= 1.6:
                self.event('clock_restore')
                self.service('/world/janus/control', 'gz.msgs.WorldControl', 'pause: false', 'clock_restore')
            elif ('clock_restore_service_ack' in self.events and 'clock_advancing_observed' not in self.events
                  and self.now-self.events['clock_restore'] >= .06):
                self.event('clock_advancing_observed')
            if 'clock_advancing_observed' in self.events:
                restored_for = self.now-self.events['clock_advancing_observed']
                if .17 <= restored_for < .57:
                    if 'unauthorized_probe' not in self.events:
                        self.event('unauthorized_probe')
                    self.send(.08, 0., 'fresh_unauthorized_request')
                elif restored_for >= .68 and 'authorization_signal_sent' not in self.events:
                    self.publish_authorize.publish(Bool(data=True))
                    self.event('authorization_signal_sent', authority='isolated_simulation_test_operator')
                if 'recovery_ack_received' in self.events and self.now-self.events['recovery_ack_received'] >= .30:
                    if 'authorized_restart_input' not in self.events:
                        self.event('authorized_restart_input')
                    self.send(.08, 0., 'fresh_post_authorization_request')
            return 'authorized_restart_input' in self.events and self.now-self.events['authorized_restart_input'] >= 1.0
        elif self.case_id == 'O05':
            if (elapsed >= 1.2 and 'regressive_odometry' not in self.events and self.latest_odometry is not None
                    and self.publish_fault_odom.get_subscription_count() > 0):
                packet = deepcopy(self.latest_odometry)
                old_stamp_ns = round((stamp_s(packet.header.stamp)-.25)*1e9)
                packet.header.stamp.sec, packet.header.stamp.nanosec = divmod(old_stamp_ns, 1_000_000_000)
                self.event('regressive_odometry', injected_capture_s=stamp_s(packet.header.stamp))
                self.publish_fault_odom.publish(packet)
            if self.publish_fault_odom is not None and self.guard and self.guard['reason'] == 'odom_regression':
                self.destroy_publisher(self.publish_fault_odom)
                self.publish_fault_odom = None
            self.send(.10, 0., 'request_through_odom_fault')
            return elapsed >= 2.7
        raise ValueError(f'Case implementation missing: {self.case_id}')

    def finish(self):
        if self.bridge_paused:
            self.change_camera_bridge(False)
        if self.clock_pause_wall is not None and 'clock_restore' not in self.events:
            self.service('/world/janus/control', 'gz.msgs.WorldControl', 'pause: false', 'cleanup_restore')
        if self.now is not None:
            self.send(0., 0., 'end')
        for service, _ in self.service_processes:
            try:
                service.wait(timeout=4)
            except subprocess.TimeoutExpired:
                service.terminate()
                self.counts['fixture_service_error'] += 1
        self.read_services()
        if self.episode_frames:
            episode = {'case_id': self.case_id, 'source': 'Gazebo Harmonic front-camera rendered RGB over ROS 2',
                       'fx_px': self.frame_focal, 'frames': self.episode_frames}
            (self.run_dir/'episode-manifest.json').write_text(json.dumps(episode, indent=2)+'\n')


def parse():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case', choices=['M01', 'M02', *QR_CASES, 'O04', 'O05', 'O08'])
    parser.add_argument('output', type=Path)
    parser.add_argument('--wall-s', type=float, default=240.)
    parser.add_argument('--camera-bridge-pid', type=int)
    parser.add_argument('--q13-mode', choices=['duplicate','regression','resume'], default='resume')
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
        node = NativeRun(args.case, args.output, trace, services, args.camera_bridge_pid, args.q13_mode)
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
        if args.case == 'Q13':
            summary['q13_mode'] = args.q13_mode
        (args.output/'execution.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
        print(json.dumps(summary, indent=2, allow_nan=False))
    return 0 if ended else 2


if __name__ == '__main__':
    raise SystemExit(main())
