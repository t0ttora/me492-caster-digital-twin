"""Lab camera decoder that preserves the baseline observation and timing contract."""
from dataclasses import replace
import json
from time import perf_counter_ns

import cv2
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import zxingcpp

from arche_experiments.ros_fields import stamp_s
from arche_experiments.vision import (
    Candidate, FrameTiming, Observation, OPTICAL_FRAME, TARGET_ID, select_marker, unpack_image,
)


def decode(image, frame):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    variants = (
        ("local_average", 1., zxingcpp.Binarizer.LocalAverage),
        ("global_histogram", 1., zxingcpp.Binarizer.GlobalHistogram),
        ("upscaled_global_histogram", 1.25, zxingcpp.Binarizer.GlobalHistogram),
    )
    for name, scale, binarizer in variants:
        scan = gray if scale == 1. else cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
        candidates = []
        for item in zxingcpp.read_barcodes(scan, formats=zxingcpp.BarcodeFormat.QRCode, binarizer=binarizer):
            position = item.position
            corners = tuple((point.x/scale, point.y/scale) for point in (
                position.top_left, position.top_right, position.bottom_right, position.bottom_left))
            try:
                candidates.append(Candidate.from_points(item.text, corners, frame.width_px, frame.height_px))
            except ValueError:
                continue
        if candidates:
            return select_marker(frame, candidates), name
    return select_marker(frame, ()), None


class LabObserver(Node):
    def __init__(self):
        super().__init__("janus_lab_qr_observer")
        self.timing = FrameTiming(.20)
        self.publisher = self.create_publisher(String, "/janus/qr/observation", 10)
        self.image_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            Image, "/janus/camera/image_raw", self.image,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT), callback_group=self.image_group)

    def image(self, msg):
        arrival = self.get_clock().now().nanoseconds / 1e9
        frame = Observation(stamp_s(msg.header.stamp), arrival, msg.width, msg.height,
                            TARGET_ID, 320., frame_id=msg.header.frame_id)
        frame = self.timing.accept(frame)
        decode_ms = None
        variant = None
        if frame.reason == "no_qr":
            if msg.header.frame_id != OPTICAL_FRAME:
                frame = replace(frame, reason="unexpected_optical_frame")
            else:
                started = perf_counter_ns()
                try:
                    pixels = unpack_image(msg.data, msg.width, msg.height, msg.step, msg.encoding)
                    frame, variant = decode(pixels, frame)
                except (ValueError, RuntimeError, cv2.error):
                    frame = replace(frame, reason="invalid_image")
                decode_ms = (perf_counter_ns()-started)/1e6
        published = self.get_clock().now().nanoseconds / 1e9
        if published < arrival:
            self.timing.clock_reset = True
            frame = replace(frame, valid=False, reason="clock_reset")
        elif frame.valid and published-frame.capture_time_s > self.timing.freshness_s:
            frame = replace(frame, valid=False, reason="stale_at_publish")
        row = {**frame.as_row(), "publish_time_s": published, "decode_ms": decode_ms,
               "decoder": "ZXing-C++ 2.2.1 Ubuntu Noble arm64", "decode_variant": variant}
        self.publisher.publish(String(data=json.dumps(row, allow_nan=False)))


def main():
    cv2.setNumThreads(1)
    rclpy.init()
    node = LabObserver()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
