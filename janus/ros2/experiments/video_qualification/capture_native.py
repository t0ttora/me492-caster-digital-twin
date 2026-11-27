#!/usr/bin/env python3
"""Save exact Gazebo camera pixels and the corresponding ROS capture metadata."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import cv2
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image

from arche_experiments.ros_fields import odometry_fields, stamp_s
from arche_experiments.vision import OPTICAL_FRAME, unpack_image


class Capture(Node):
    def __init__(self, output, count):
        super().__init__("video_qualification_capture")
        self.output, self.count = output, count
        self.rows = []
        self.clock_s = None
        self.calibration = None
        self.world_pose = None
        self.create_subscription(Clock, "/clock", self.clock, qos_profile_sensor_data)
        self.create_subscription(Image, "/janus/camera/image_raw", self.image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/janus/camera/camera_info",
                                 self.info, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/janus/ground_truth", self.pose, qos_profile_sensor_data)

    def clock(self, msg):
        self.clock_s = stamp_s(msg.clock)

    def info(self, msg):
        self.calibration = {"frame_id": msg.header.frame_id, "matrix": list(msg.k),
                            "distortion": list(msg.d), "width": msg.width, "height": msg.height}

    def pose(self, msg):
        try:
            self.world_pose = odometry_fields(msg, "map")
        except ValueError:
            return

    def image(self, msg):
        if len(self.rows) >= self.count or self.calibration is None or self.world_pose is None:
            return
        if msg.encoding != "rgb8" or msg.header.frame_id != OPTICAL_FRAME:
            raise ValueError(f"Unexpected Gazebo image: {msg.encoding}, {msg.header.frame_id}")
        pixels = unpack_image(msg.data, msg.width, msg.height, msg.step, msg.encoding)
        path = self.output / f"{len(self.rows):03d}.png"
        if not cv2.imwrite(str(path), pixels):
            raise OSError(path)
        restored = cv2.imread(str(path), cv2.IMREAD_COLOR)
        restored_rgb = cv2.cvtColor(restored, cv2.COLOR_BGR2RGB).tobytes()
        source_rgb = b"".join(bytes(msg.data[i:i + msg.width * 3])
                              for i in range(0, len(msg.data), msg.step))
        if restored_rgb != source_rgb:
            raise ValueError("PNG failed lossless round trip")
        self.rows.append({
            "png": path.name, "png_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "rgb_sha256": hashlib.sha256(source_rgb).hexdigest(),
            "image_topic": "/janus/camera/image_raw", "encoding": msg.encoding,
            "capture_time_s": stamp_s(msg.header.stamp), "clock_on_callback_s": self.clock_s,
            "pose_latest_on_callback": self.world_pose, "camera_info": self.calibration,
        })


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("out", type=Path)
    cli.add_argument("--count", type=int, default=12)
    args = cli.parse_args()
    args.out.mkdir(parents=True)
    rclpy.init()
    node = Capture(args.out, args.count)
    end = time.monotonic() + 25
    try:
        while len(node.rows) < args.count and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        (args.out / "capture.json").write_text(json.dumps(node.rows, indent=2) + "\n")
        node.destroy_node()
        rclpy.try_shutdown()
    if len(node.rows) != args.count:
        raise RuntimeError(f"Captured {len(node.rows)}/{args.count} frames")
    print(f"Saved {len(node.rows)} native RGB frames to {args.out}")


if __name__ == "__main__":
    main()
