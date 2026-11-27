#!/usr/bin/env python3
"""Q04 native camera and wheel-odometry staging, with independent truth logging."""

import argparse
from collections import Counter
from dataclasses import fields
import json
import math
from pathlib import Path
from statistics import median
import time

import cv2
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from arche_experiments.ros_fields import odometry_fields, stamp_s, twist_fields
from arche_experiments.vision import CenteringGate, Observation, unpack_image


OBSERVATION_FIELDS = {field.name for field in fields(Observation)}
QR_ID = "JANUS-DOCK-01"
QR_SIDE_M = .24 * 21 / 29
CAMERA_FORWARD_M = .25
QR_FRESHNESS_S = .20
ODOM_MAX_AGE_S = .12
STAGE_STANDOFF_M = .74
OPTICAL_STOP_TARGET_M = .260
STOP_DWELL_S = .25


def wrap(angle):
    return math.remainder(angle, 2 * math.pi)


def clamp(value, limit):
    return max(-limit, min(limit, value))


def marker_center(frame, fx, fy, cx):
    """Reconstruct both marker edges in the level camera frame from pixel scale."""
    tl, tr, br, bl = frame.corners_px
    depths = []
    laterals = []
    for top, bottom in ((tl, bl), (tr, br)):
        height = bottom[1] - top[1]
        if height <= 1:
            raise ValueError("nonvertical or degenerate marker edge")
        depth = fy * QR_SIDE_M / height
        depths.append(depth)
        laterals.append((cx - (top[0] + bottom[0]) / 2) * depth / fx)
    return sum(depths) / 2, sum(laterals) / 2, math.atan2(
        depths[1] - depths[0], laterals[0] - laterals[1]
    )


def stopped(pose):
    return abs(pose["v_mps"]) <= .02 and abs(pose["yaw_rate_radps"]) <= .05


class Q04Stage(Node):
    def __init__(self, run_dir, trace, fixture):
        super().__init__("janus_q04_mechanical")
        self.run_dir = run_dir
        self.trace = trace
        self.counts = Counter()
        self.now = None
        self.poll_after = None
        self.started = None
        self.last_clock_log = None
        self.pose = None
        self.pose_history = []
        self.frame = None
        self.intrinsics = None
        self.gate = CenteringGate()
        self.phase = "measure_marker"
        self.station_estimates = {}
        self.stage_goal = None
        self.dwell_start = None
        self.held_from = None
        self.last_frame = None
        self.next_snapshot = "initial"
        self.station_yaw = wrap(fixture["marker_xyz_rpy"][5] - fixture["spawn_xyz_rpy"][5])
        self.publisher = self.create_publisher(Twist, "/janus/cmd_vel/request", 10)

        subscriptions = (
            (Clock, "/clock", self.clock),
            (Odometry, "/odom", self.odom),
            (Odometry, "/janus/ground_truth", self.log_truth),
            (Twist, "/cmd_vel", self.guarded),
            (String, "/janus/command_guard/status", self.guard_status),
            (CameraInfo, "/janus/camera/camera_info", self.camera_info),
            (Image, "/janus/camera/image_raw", self.image),
            (String, "/janus/qr/observation", self.observation),
        )
        for message, topic, callback in subscriptions:
            self.create_subscription(message, topic, callback, qos_profile_sensor_data)

    def log(self, source, **values):
        self.counts[source] += 1
        row = {"source": source, "read_time_s": self.now, **values}
        self.trace.write(json.dumps(row, allow_nan=False) + "\n")

    def clock(self, msg):
        tick = stamp_s(msg.clock)
        if self.now is not None and tick < self.now:
            raise RuntimeError("simulation clock regressed")
        self.now = tick
        if self.last_clock_log is None or tick - self.last_clock_log >= .05:
            self.last_clock_log = tick
            self.log("clock", time_s=tick)

    def odom(self, msg):
        self.pose = odometry_fields(msg, "odom")
        self.pose_history.append(self.pose)
        while self.pose_history[0]["time_s"] < self.pose["time_s"] - .4:
            self.pose_history.pop(0)
        self.log("odom", **self.pose)

    def log_truth(self, msg):
        # This callback never assigns a pose used anywhere in control.
        self.log("truth", **odometry_fields(msg, "map"))

    def guarded(self, msg):
        self.log("guard_output", time_s=self.now, **twist_fields(msg))

    def guard_status(self, msg):
        self.log("guard_status", **json.loads(msg.data))

    def camera_info(self, msg):
        if self.intrinsics is None and msg.k[0] > 0 and msg.k[4] > 0:
            self.intrinsics = (msg.k[0], msg.k[4], msg.k[2])
            self.log("intrinsics", matrix=list(msg.k), frame_id=msg.header.frame_id,
                     width_px=msg.width, height_px=msg.height)

    def image(self, msg):
        self.last_frame = msg
        if self.next_snapshot is not None:
            self.save_image(self.next_snapshot)
            self.next_snapshot = None

    def save_image(self, label):
        msg = self.last_frame
        if msg is None:
            return
        pixels = unpack_image(msg.data, msg.width, msg.height, msg.step, msg.encoding)
        path = self.run_dir / (label + "-rendered.png")
        if not cv2.imwrite(str(path), pixels):
            raise OSError(f"could not write {path.name}")
        self.log("image", file=path.name, capture_time_s=stamp_s(msg.header.stamp))

    def observation(self, msg):
        row = json.loads(msg.data)
        self.frame = Observation(**{key: row[key] for key in OBSERVATION_FIELDS if key in row})
        row["observation_source"] = row.pop("source")
        self.log("qr", **row)

    def send(self, speed, yaw, reason):
        command = Twist()
        command.linear.x, command.angular.z = speed, yaw
        self.publisher.publish(command)
        self.log("request", time_s=self.now, v_mps=speed, yaw_rate_radps=yaw, phase=self.phase,
                 reason=reason, capture_time_s=self.frame.capture_time_s if self.frame else None,
                 normalized_error=self.frame.error if self.frame else None)

    def switch(self, phase):
        self.phase = phase
        self.dwell_start = None
        self.log("event", event=phase, time_s=self.now)

    def dwell(self, condition):
        if condition:
            if self.dwell_start is None:
                self.dwell_start = self.now
            return self.now - self.dwell_start >= STOP_DWELL_S
        self.dwell_start = None
        return False

    def estimate_station(self):
        capture = self.frame.capture_time_s
        if capture in self.station_estimates:
            return
        pose = min(self.pose_history, key=lambda row: abs(row["time_s"] - capture))
        if abs(pose["time_s"] - capture) > .04 or not stopped(pose):
            return
        forward, left, heading = marker_center(self.frame, *self.intrinsics)
        if not .35 < forward < 2 or abs(left) > .7:
            return
        angle = pose["yaw_rad"]
        x = pose["x_m"] + (CAMERA_FORWARD_M + forward) * math.cos(angle) - left * math.sin(angle)
        y = pose["y_m"] + (CAMERA_FORWARD_M + forward) * math.sin(angle) + left * math.cos(angle)
        self.station_estimates[capture] = (x, y)
        self.log("marker_pose", capture_time_s=capture, odom_time_s=pose["time_s"],
                 marker_x_odom_m=x, marker_y_odom_m=y, observed_yaw_odom_rad=wrap(angle + heading))
        if len(self.station_estimates) >= 7:
            center_x = median(sample[0] for sample in self.station_estimates.values())
            center_y = median(sample[1] for sample in self.station_estimates.values())
            self.stage_goal = (center_x - STAGE_STANDOFF_M * math.cos(self.station_yaw),
                               center_y - STAGE_STANDOFF_M * math.sin(self.station_yaw))
            self.log("staging_goal", center_x_m=center_x, center_y_m=center_y,
                     x_m=self.stage_goal[0], y_m=self.stage_goal[1], heading_rad=self.station_yaw,
                     heading_source="declared fixture marker yaw relative to declared initial pose")
            self.switch("stage_turn_to_position")

    def stage_command(self):
        x, y = self.pose["x_m"], self.pose["y_m"]
        dx, dy = self.stage_goal[0] - x, self.stage_goal[1] - y
        yaw = self.pose["yaw_rad"]
        distance = math.hypot(dx, dy)
        bearing_error = wrap(math.atan2(dy, dx) - yaw)
        if self.phase == "stage_turn_to_position":
            if self.dwell(abs(bearing_error) <= .025 and stopped(self.pose)):
                self.switch("stage_translate")
            self.send(0., 0. if abs(bearing_error) <= .020 else clamp(2 * bearing_error, .34),
                      "stage_bearing")
        elif self.phase == "stage_translate":
            if self.dwell(distance <= .010 and stopped(self.pose)):
                self.switch("stage_turn_to_dock")
            if distance <= .010:
                self.send(0., 0., "stage_position_hold")
            else:
                speed = min(.10, max(.018, distance)) if abs(bearing_error) < .16 else 0.
                self.send(speed, clamp(2.5 * bearing_error, .30), "stage_position_track")
        else:
            error = wrap(self.station_yaw - yaw)
            if self.dwell(abs(error) <= .020 and stopped(self.pose)):
                self.switch("optical_center")
                self.next_snapshot = "stage"
            self.send(0., 0. if abs(error) <= .016 else clamp(2 * error, .34), "stage_dock_heading")

    def optical_command(self):
        frame = self.frame
        allowed = self.gate.update(self.now, frame)
        error = frame.error
        turn = 0. if abs(error) <= .004 or (not allowed and abs(error) <= .02) else clamp(-1.8 * error, .35)
        depth, _, _ = marker_center(frame, *self.intrinsics)
        if not allowed:
            self.send(0., turn, "center_fresh_dwell")
        elif depth <= OPTICAL_STOP_TARGET_M:
            if abs(error) <= .02:
                self.switch("terminal_hold")
                self.held_from = self.now
                self.next_snapshot = "standoff"
                self.send(0., 0., "optical_standoff")
            else:
                self.send(0., turn, "center_at_standoff")
        else:
            speed = min(.10, max(.018, .8 * (depth - OPTICAL_STOP_TARGET_M)))
            self.send(speed, turn, "centered_optical_advance")

    def tick(self):
        if self.now is None or self.pose is None or self.intrinsics is None:
            return False
        if self.poll_after is not None and self.now < self.poll_after:
            return False
        self.poll_after = self.now + .025
        if self.started is None:
            self.started = self.now
            self.log("event", event="case_start", time_s=self.now)
        fresh = (self.frame is not None and self.frame.valid and self.frame.id == QR_ID
                 and self.frame.source == "decoded_image"
                 and 0 <= self.now - self.frame.capture_time_s <= QR_FRESHNESS_S)
        if self.phase == "terminal_hold":
            self.send(0., 0., "hold_measured_optical_standoff")
            if self.now - self.held_from >= .65 and fresh and stopped(self.pose):
                self.save_image("terminal")
                self.log("event", event="case_end", time_s=self.now)
                return True
        elif not fresh or not -.025 <= self.now - self.pose["time_s"] <= ODOM_MAX_AGE_S:
            self.gate.update(self.now, self.frame)
            self.dwell_start = None
            self.send(0., 0., "fresh_observation_and_odometry_required")
        elif self.phase == "measure_marker":
            self.send(0., 0., "collect_station_estimates")
            self.estimate_station()
        elif self.phase.startswith("stage_"):
            self.stage_command()
        else:
            self.optical_command()
        return False


def main():
    args = argparse.ArgumentParser(description=__doc__)
    args.add_argument("output", type=Path)
    args.add_argument("--wall-s", type=float, default=125.)
    params = args.parse_args()
    fixture = json.loads((params.output / "fixture.json").read_text())
    assert fixture["case_id"] == "Q04" and fixture["target_u_px"] == 320.
    rclpy.init()
    cv2.setNumThreads(1)
    end = time.monotonic() + params.wall_s
    finished = False
    with (params.output / "telemetry.jsonl").open("w", buffering=1) as trace:
        run = Q04Stage(params.output, trace, fixture)
        try:
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(run, timeout_sec=.008)
                if run.tick():
                    finished = True
                    break
        finally:
            run.send(0., 0., "controller_exit")
            topics = ("/clock", "/odom", "/janus/ground_truth", "/cmd_vel", "/janus/cmd_vel/request",
                      "/janus/qr/observation")
            summary = {"completed": finished, "phase": run.phase, "samples": dict(run.counts),
                       "publishers": {topic: run.count_publishers(topic) for topic in topics},
                       "station_goal": run.stage_goal, "nominal_staging_heading_odom_rad": run.station_yaw}
            (params.output / "execution.json").write_text(json.dumps(summary, indent=2) + "\n")
            run.destroy_node()
            rclpy.try_shutdown()
    return 0 if finished else 1


if __name__ == "__main__":
    raise SystemExit(main())
