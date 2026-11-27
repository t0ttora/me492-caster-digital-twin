#!/usr/bin/env python3
"""Replay retained camera pixels through the unmodified ROS observer and live guard."""
import argparse
from array import array
from concurrent.futures import ThreadPoolExecutor
from dataclasses import fields
import hashlib
from importlib.machinery import SourceFileLoader
from importlib.util import module_from_spec, spec_from_loader
import json
import math
from pathlib import Path
import signal
from threading import Thread
from time import monotonic, monotonic_ns, sleep

import cv2
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String

from arche_experiments.ros_fields import odometry_fields, stamp_s, twist_fields
from arche_experiments.vision import CenteringGate, Observation, OPTICAL_FRAME

PERIOD = 0.05
TICK = 0.02
VELOCITY = 0.08
PREFACE = 1.80
DISTURBANCE = 1.20
RECOVERY = 1.00
INTERMISSION = 0.50
TOTAL = PREFACE + DISTURBANCE + RECOVERY + INTERMISSION

TRIALS = (
    ("Q09", "two_ids", "duplicate_ids", 0.0),
    ("Q10", "clean", "mild_glare", 0.0),
    ("Q10", "clean", "hotspot", 0.0),
    ("Q10", "clean", "reflection", 0.0),
    ("Q10", "clean", "exposure_mild", 0.0),
    ("Q10", "clean", "exposure_clip", 0.0),
    ("Q10", "clean", "exposure_whiteout", 0.0),
    ("Q11", "clean", "low_light", 0.0),
    ("Q11", "clean", "motion_blur", 0.0),
    ("Q11", "clean", "low_light_motion_blur", 0.0),
    ("Q11", "clean", "motion_blur", 0.0),
    ("Q11", "clean", "low_light_motion_blur", 0.0),
    ("Q11", "low_light", "low_light", 0.28),
    ("Q12", "clean", "occlusion", 0.0),
    ("Q12", "near_full_view", "edge_crop", 0.0),
    ("Q12", "near_full_view", "native_near_edge", 0.0),
)


def ros_stamp(seconds):
    ns = round(seconds * 1e9)
    return Time(sec=ns // 10**9, nanosec=ns % 10**9)


def prepare_images(root, manifest):
    images = {}
    cache = {}
    for name, fixture in manifest["fixtures"].items():
        images[name] = []
        for item in fixture["frames"]:
            path = root / item["path"]
            if hashlib.sha256(path.read_bytes()).hexdigest() != item["png_sha256"]:
                raise ValueError(f"Unexpected fixture digest: {path}")
            if item["png_sha256"] not in cache:
                pixels = cv2.imread(str(path), cv2.IMREAD_COLOR)
                if pixels is None:
                    raise OSError(path)
                h, w, _ = pixels.shape
                rgb = cv2.cvtColor(pixels, cv2.COLOR_BGR2RGB).tobytes()
                msg = Image(height=h, width=w, encoding="rgb8", step=w*3, is_bigendian=0,
                            data=array("B", rgb))
                msg.header.frame_id = OPTICAL_FRAME
                cache[item["png_sha256"]] = (msg, hashlib.sha256(rgb).hexdigest())
            msg, rgb_digest = cache[item["png_sha256"]]
            images[name].append((msg, item, rgb_digest))
    return images


def schedule(period):
    captures, trials = [], []
    for i, (case, preface, stimulus, delay) in enumerate(TRIALS):
        start = round(i * TOTAL, 6)
        trials.append({"trial": i, "case": case, "preface": preface, "stimulus": stimulus,
                       "start_s": start, "disturbance_s": round(start + PREFACE, 6),
                       "recovery_s": round(start + PREFACE + DISTURBANCE, 6),
                       "intermission_s": round(start + PREFACE + DISTURBANCE + RECOVERY, 6),
                       "end_s": round(start + TOTAL, 6), "injected_delay_s": delay})
        for phase, variant, offset, duration, latency in (
                ("preface", preface, 0, PREFACE, 0),
                ("disturbance", stimulus, PREFACE, DISTURBANCE, delay),
                ("recovery", preface, PREFACE + DISTURBANCE + delay, RECOVERY - delay, 0)):
            for frame_index in range(round(duration / period)):
                stamp = round(start + offset + frame_index * period, 6)
                captures.append({"trial": i, "case": case, "variant": variant, "phase": phase,
                                 "fixture_frame_index_requested": frame_index,
                                 "relative_capture_s": stamp, "relative_due_s": round(stamp + latency, 6)})
    return captures, trials


class Replay(Node):
    def __init__(self, images, captures, trials, stream, direct=None):
        super().__init__("video_qualification_replay")
        self.images, self.captures, self.trials, self.stream = images, captures, trials, stream
        self.elapsed_clock = None
        self.odom_seen = False
        self.ground_seen = False
        self.origin = None
        self.index = 0
        self.current_trial = None
        self.gate = CenteringGate()
        self.observation = None
        self.sent = {}
        self.finished = False
        self.direct = direct
        self.image_pub = (None if direct else
                          self.create_publisher(Image, "/janus/camera/image_raw", qos_profile_sensor_data))
        self.request_pub = self.create_publisher(Twist, "/janus/cmd_vel/request", 10)
        self.create_subscription(ClockMessage, "/clock", self.on_clock, qos_profile_sensor_data)
        self.create_subscription(String, "/janus/qr/observation", self.on_observation, 10)
        self.create_subscription(String, "/janus/command_guard/status", self.on_status, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_guarded, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/janus/ground_truth", self.on_ground, qos_profile_sensor_data)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(TICK, self.on_timer, clock=self.steady)
        self.row_fields = {field.name for field in fields(Observation)}

    def log(self, kind, **data):
        self.stream.write(json.dumps({"kind": kind, "clock_s": self.elapsed_clock,
                         "wall_ns": monotonic_ns(), **data}, allow_nan=False) + "\n")

    def on_clock(self, msg):
        self.elapsed_clock = stamp_s(msg.clock)

    def on_status(self, msg):
        self.log("guard_status", **json.loads(msg.data))

    def on_guarded(self, msg):
        if self.origin is not None:
            self.log("guard_output", **twist_fields(msg))

    def telemetry(self, msg, parent, kind):
        try:
            row = odometry_fields(msg, parent)
        except ValueError as exc:
            self.log("telemetry_error", topic=kind, reason=str(exc),
                     parent=msg.header.frame_id, child=msg.child_frame_id)
            return False
        if self.origin is not None:
            velocity = msg.twist.twist.linear
            self.log(kind, **row, planar_speed_mps=math.hypot(velocity.x, velocity.y))
        return True

    def on_odom(self, msg):
        self.odom_seen = self.telemetry(msg, "odom", "odom")

    def on_ground(self, msg):
        self.ground_seen = self.telemetry(msg, "map", "ground_truth")

    def on_observation(self, msg):
        row = json.loads(msg.data)
        key = round(row["capture_time_s"] * 1e9)
        frame_id = self.sent.get(key)
        self.log("observation", frame_sequence=frame_id, **row)
        if self.origin is None or frame_id is None or self.elapsed_clock is None:
            return
        fixture = self.captures[frame_id]
        if fixture["trial"] != self.current_trial:
            return
        obs = Observation(**{name: row[name] for name in self.row_fields})
        # The observer has its own sim clock; never feed a consumer time earlier
        # than its published observation if local /clock delivery lags a tick.
        now = max(row["publish_time_s"], self.elapsed_clock)
        self.observation = obs
        self.issue(now, self.gate.update(now, obs), f'frame:{row["reason"]}', frame_id)

    def issue(self, now, forward, trigger, frame_id=None, force_zero=False):
        if self.current_trial is None:
            return
        phase = self.trials[self.current_trial]
        enabled = not force_zero and now - self.origin < phase["intermission_s"]
        request = Twist()
        request.linear.x = VELOCITY if forward and enabled else 0.0
        self.request_pub.publish(request)
        observation = self.observation
        if not enabled:
            reason = "intermission"
        elif observation is None:
            reason = "waiting_first_frame"
        elif now - observation.capture_time_s > 0.20:
            reason = "expired_at_control"
        elif not observation.valid:
            reason = observation.reason
        elif not forward:
            reason = "center_dwell_or_band"
        else:
            reason = "fresh_target_centered"
        self.log("request", trial=self.current_trial, frame_sequence=frame_id, now_s=now,
                 trigger=trigger, reason=reason, gate_forward_allowed=forward and enabled,
                 selected_id=observation.id if observation else None, **twist_fields(request))

    def publish_frame(self, sequence, spec):
        bank = self.images[spec["variant"]]
        msg, item, digest = bank[spec["fixture_frame_index_requested"] % len(bank)]
        capture = self.origin + spec["relative_capture_s"]
        msg.header.stamp = ros_stamp(capture)
        exact_capture = stamp_s(msg.header.stamp)
        self.sent[round(exact_capture * 1e9)] = sequence
        self.log("image_sent", frame_sequence=sequence, **spec, capture_time_s=exact_capture,
                 image_delivery="in_process_observer_callback" if self.direct else "ROS2_sensor_topic",
                 fixture_frame_index=item["frame_index"], fixture_path=item["path"],
                 png_sha256=item["png_sha256"], rgb8_sha256=digest,
                 native_capture_time_s=(item["native_capture"] or {}).get("capture_time_s"))
        if self.direct:
            # Banked images are reused. Queue a message with a dedicated timestamp.
            queued = Image(header=Header(stamp=ros_stamp(capture), frame_id=OPTICAL_FRAME),
                           height=msg.height, width=msg.width, step=msg.step,
                           encoding=msg.encoding, is_bigendian=msg.is_bigendian, data=msg.data)
            self.direct.submit(queued)
        else:
            self.image_pub.publish(msg)

    def on_timer(self):
        if self.elapsed_clock is None or not self.ground_seen or not self.odom_seen:
            return
        if self.origin is None:
            image_subs = self.image_pub.get_subscription_count() if self.image_pub else 0
            image_ready = self.direct.ready() if self.direct else image_subs > 0
            if not image_ready or not self.request_pub.get_subscription_count():
                return
            self.origin = round(self.elapsed_clock + 0.5, 6)
            self.log("origin", simulation_origin_s=self.origin,
                     replay_image_subscriptions=image_subs,
                     image_delivery="in_process_observer_callback" if self.direct else "ROS2_sensor_topic",
                     guard_request_subscriptions=self.request_pub.get_subscription_count())
        relative = self.elapsed_clock - self.origin
        if relative < 0:
            return
        if relative >= self.trials[-1]["end_s"]:
            self.issue(self.elapsed_clock, False, "end", force_zero=True)
            self.finished = True
            return
        trial = min(int((relative + 1e-8) / TOTAL), len(self.trials)-1)
        if self.current_trial != trial:
            self.current_trial = trial
            self.gate = CenteringGate()
            self.observation = None
            self.log("start_trial", **self.trials[trial])
        while self.index < len(self.captures) and self.captures[self.index]["relative_due_s"] <= relative:
            spec = self.captures[self.index]
            self.publish_frame(self.index, spec)
            self.index += 1
        self.issue(self.elapsed_clock, self.gate.update(self.elapsed_clock), "timer")


class CallbackDelivery:
    """Call the pinned production image handler serially with retained RGB8 messages."""

    def __init__(self, source):
        loader = SourceFileLoader("janus_pinned_qr_observer", str(source))
        spec = spec_from_loader(loader.name, loader)
        module = module_from_spec(spec)
        loader.exec_module(module)
        self.node = module.QrObserver()
        self.node.destroy_subscription(self.node.subscription)
        self.node.set_parameters([Parameter("use_sim_time", value=True)])
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = Thread(target=self.executor.spin, name="observer-clock", daemon=True)
        self.spin_thread.start()
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="observer-image")
        self.completed = []

    def submit(self, message):
        self.completed.append(self.worker.submit(self.deliver, message))

    def deliver(self, message):
        # The driver and observer receive /clock separately. A synthetic replay
        # must not present a future-dated image just because one callback lagged.
        stamp = message.header.stamp
        capture_ns = stamp.sec * 10**9 + stamp.nanosec
        deadline = monotonic() + 1.0
        while self.node.get_clock().now().nanoseconds < capture_ns:
            if monotonic() >= deadline:
                raise RuntimeError("Observer clock did not reach replay capture timestamp")
            sleep(0.001)
        self.node.image(message)

    def ready(self):
        return self.node.get_clock().now().nanoseconds > 0 and self.node.publisher.get_subscription_count() > 0

    def close(self):
        self.worker.shutdown(wait=True)
        try:
            for work in self.completed:
                work.result()
        finally:
            self.executor.shutdown()
            self.spin_thread.join()
            self.node.destroy_node()


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("fixtures", type=Path)
    cli.add_argument("out", type=Path)
    cli.add_argument("--period", type=float, choices=(0.05, 0.10), default=PERIOD)
    cli.add_argument("--direct-observer-source", type=Path)
    cli.add_argument("--timeout-wall-s", type=float, default=720)
    args = cli.parse_args()
    if not math.isfinite(args.timeout_wall_s) or args.timeout_wall_s <= 0:
        cli.error("--timeout-wall-s must be finite and positive")
    args.out.mkdir(parents=True)
    cv2.setNumThreads(1)
    manifest = json.loads((args.fixtures / "manifest.json").read_text())
    images = prepare_images(args.fixtures, manifest)
    captures, trials = schedule(args.period)
    (args.out / "schedule.json").write_text(json.dumps({
        "clock": "Gazebo /clock for sim, Linux monotonic for wall measurements",
        "image_delivery": ("in_process_observer_callback" if args.direct_observer_source else "ROS2_sensor_topic"),
        "opencv_version": cv2.__version__, "driver_opencv_threads": cv2.getNumThreads(),
        "observer_opencv_threads": 1,
        "requested_velocity_mps": VELOCITY, "period_s": args.period, "guard_request_tick_wall_s": TICK,
        "wall_timeout_s": args.timeout_wall_s,
        "freshness_s": 0.2, "center_dwell_s": 0.4, "suite": trials, "expected_frames": len(captures),
    }, indent=2) + "\n")
    stopping = False

    def stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    start = monotonic()
    direct = None
    try:
        if args.direct_observer_source:
            direct = CallbackDelivery(args.direct_observer_source)
        with (args.out / "trace.jsonl").open("w", buffering=1) as stream:
            node = Replay(images, captures, trials, stream, direct)
            try:
                while not (node.finished or stopping) and monotonic() - start < args.timeout_wall_s:
                    rclpy.spin_once(node, timeout_sec=.1)
                if node.index != len(captures) or not node.finished:
                    reason = "operator stop" if stopping else "timeout"
                    raise RuntimeError(f"Incomplete replay {node.index}/{len(captures)}: {reason}")
            finally:
                for _ in range(6):
                    node.request_pub.publish(Twist())
                    rclpy.spin_once(node, timeout_sec=.02)
                node.destroy_node()
    finally:
        try:
            if direct:
                direct.close()
        finally:
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
