from hashlib import sha256
import json
import math
from pathlib import Path
from types import SimpleNamespace as Message
import tempfile
import unittest

import cv2
import numpy as np

from arche_experiments.commands import TORQUE, VELOCITY, CommandSeries, load_commands
from arche_experiments.fixtures import centering_fixture, guard_fixture, qr_fixtures, timing_fixture
from arche_experiments.replay import ideal_velocity_replay
from arche_experiments.ros_fields import JOINTS, joint_fields, odometry_fields
from arche_experiments.trace_metrics import measure_trace
from arche_experiments.vision import Decoder, Observation, TARGET_ID, unpack_image

try:
    from arche_experiments.vision_zbar import ZBarDecoder
except (ImportError, OSError):
    ZBarDecoder = None


class CommandTests(unittest.TestCase):
    def test_zoh_boundary_and_end(self):
        series = CommandSeries(VELOCITY, (0, 0.13, 1), ((0.1, 0), (-0.2, 0), (0, 0)))
        self.assertEqual(series.hold(0.129), (0.1, 0))
        self.assertEqual(series.hold(0.13), (-0.2, 0))
        self.assertEqual(series.hold(1), (0, 0))
        with self.assertRaises(ValueError):
            series.hold(1.001)

    def test_straight_reverse_and_turn(self):
        series = CommandSeries(VELOCITY, (0, 1, 2, 3),
                               ((0.1, 0), (-0.1, 0), (0, math.pi / 2), (0, 0)))
        rows = list(ideal_velocity_replay(series, 0.3, 0.075, 0.38))
        stamps = {row["time_s"]: row for row in rows}
        self.assertAlmostEqual(stamps[1]["x_m"], 0.1)
        self.assertLess(stamps[1]["ideal_left_radps"], 0)
        self.assertLess(stamps[1]["ideal_right_radps"], 0)
        self.assertAlmostEqual(stamps[2]["x_m"], 0)
        self.assertLess(stamps[2]["ideal_left_radps"], 0)
        self.assertGreater(stamps[2]["ideal_right_radps"], 0)
        self.assertAlmostEqual(stamps[3]["yaw_rad"], math.pi / 2)
        self.assertAlmostEqual(stamps[3]["integrated_abs_distance_m"], 0.2)

    def test_exact_arc_and_origin(self):
        series = CommandSeries(VELOCITY, (0, 1), ((0.2, math.pi / 2), (0, 0)))
        last = list(ideal_velocity_replay(series, 0.3, 0.075, 0.38, (1, 2, math.pi / 2)))[-1]
        self.assertAlmostEqual(last["x_m"], 1 - 0.4 / math.pi)
        self.assertAlmostEqual(last["y_m"], 2 + 0.4 / math.pi)
        self.assertAlmostEqual(last["yaw_rad"], math.pi)

    def test_torque_is_never_interpreted_as_speed(self):
        with self.assertRaisesRegex(ValueError, "effort-actuated plant"):
            next(ideal_velocity_replay(CommandSeries(TORQUE, (0, 1), ((0, 0), (0, 0))),
                                       0.02, 0.075, 0.38))

    def test_csv_validation(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "commands.csv"
            for invalid in (["1,0,0", "2,0,0"], ["0,0,0", "0,0,0"],
                            ["0,0,0", "1,nan,0"], ["0,0,0", "1,0,0,0"], ["0,0,0"]):
                with self.subTest(invalid=invalid):
                    path.write_text(",".join(VELOCITY) + "\n" + "\n".join(invalid))
                    with self.assertRaises(ValueError):
                        load_commands(path)
            path.write_text(",".join(TORQUE) + "\n0,0,0\n1,2,-2\n")
            self.assertEqual(load_commands(path).hold(1), (2, -2))


class VisionTests(unittest.TestCase):
    @unittest.skipIf(ZBarDecoder is None, 'optional native ZBar backend unavailable')
    def test_zbar_backend_does_not_weaken_wrong_and_ambiguous_gate(self):
        for case, variant, image, target, valid, reason, _ in qr_fixtures():
            if case not in {'Q08', 'Q09'}:
                continue
            with self.subTest(case=case, variant=variant):
                observed = ZBarDecoder().decode(image, Observation(0,0,640,480,TARGET_ID,target))
                self.assertEqual(observed.valid, valid)
                self.assertEqual(observed.reason, reason)

    def test_rendered_marker_recovery_preserves_detected_pixel_geometry(self):
        image = cv2.imread(str(Path(__file__).parent / 'fixtures/gazebo_aliasing_rgb.png'))
        decoded = Decoder().decode(image, Observation(0, 0, 640, 480, TARGET_ID, 320.0))
        self.assertTrue(decoded.valid, decoded.reason)
        self.assertEqual(decoded.id, TARGET_ID)
        self.assertEqual(len(decoded.corners_px), 4)
        self.assertAlmostEqual(decoded.center_u_px, 320.0, delta=2.0)
        self.assertGreater(decoded.quality_area_ratio, 0.015)

    def test_real_decoding_cases(self):
        decode = Decoder()
        for case, variant, image, target, valid, reason, sign in qr_fixtures():
            with self.subTest(case=case, variant=variant):
                frame = decode.decode(image, Observation(0, 0, 640, 480, TARGET_ID, target))
                self.assertEqual(frame.valid, valid)
                if reason:
                    self.assertEqual(frame.reason, reason)
                if sign is not None:
                    self.assertEqual(-np.sign(frame.error), sign)
                if case == "Q09":
                    self.assertEqual(len(frame.candidates), 2)
                if variant == "rotated_shifted_target":
                    u, v = frame.corners_px[0]
                    self.assertGreater(u, target)
                    self.assertLess(v, 240)

    def test_decoder_exception_is_invalid_frame(self):
        decode = Decoder()

        def fail(*_):
            raise cv2.error("synthetic failure from OpenCV detectAndDecodeMulti")

        decode._candidates = fail
        frame = decode.decode(np.zeros((12, 12), dtype=np.uint8), Observation(0, 0, 12, 12, TARGET_ID, 6))
        self.assertFalse(frame.valid)
        self.assertEqual(frame.reason, "decoder_error")

    def test_ros_image_stride_and_rgb_channel_order(self):
        data = bytes([1, 2, 3, 20, 30, 40, 99, 99, 4, 5, 6, 50, 60, 70, 99, 99])
        img = unpack_image(data, 2, 2, 8, "rgb8")
        self.assertEqual(img.tolist(), [[[3, 2, 1], [40, 30, 20]], [[6, 5, 4], [70, 60, 50]]])
        with self.assertRaises(ValueError):
            unpack_image(data[:-1], 2, 2, 8, "rgb8")
        with self.assertRaises(ValueError):
            unpack_image(data, 2, 2, 8, "16UC1")

    def test_clock_and_frame_replay(self):
        self.assertTrue(all(row["passed"] for row in timing_fixture()))

    def test_centering_dwell_hysteresis_and_stream_cut(self):
        rows = list(centering_fixture())
        self.assertTrue(all(row["passed"] for row in rows), rows)
        self.assertTrue(any(row["gate_forward_allowed"] for row in rows))
        self.assertFalse(rows[-1]["gate_forward_allowed"])


class StateTests(unittest.TestCase):
    def test_guard_variants(self):
        for variant in ("command_timeout", "operator_abort", "odom_timeout", "odom_regression",
                        "clock_stall", "clock_reset", "clock_recovery", "odom_recovery", "restart_recovery"):
            with self.subTest(variant=variant):
                self.assertTrue(all(row["passed"] for row in guard_fixture(variant)))

    def test_odometry_extraction_and_quaternion(self):
        header = Message(stamp=Message(sec=1, nanosec=20_000_000), frame_id="odom")
        pose = Message(position=Message(x=1, y=2, z=0.0),
                       orientation=Message(x=0, y=0, z=math.sqrt(0.5), w=math.sqrt(0.5)))
        twist = Message(linear=Message(x=0.1), angular=Message(z=0.3))
        msg = Message(header=header, child_frame_id="base_link", pose=Message(pose=pose),
                      twist=Message(twist=twist))
        row = odometry_fields(msg, "odom")
        self.assertEqual(row["time_s"], 1.02)
        self.assertAlmostEqual(row["yaw_rad"], math.pi / 2)
        self.assertAlmostEqual(row["yaw_rate_radps"], 0.3)
        with self.assertRaises(ValueError):
            odometry_fields(msg, "map")
        pose.orientation = Message(x=0, y=0, z=0, w=0)
        with self.assertRaises(ValueError):
            odometry_fields(msg, "odom")

    def test_joint_extraction_uses_names_and_omits_effort(self):
        msg = Message(name=list(reversed(JOINTS)), position=[10, 20, 30, 40],
                      velocity=[1, 2, 3, 4], effort=[0, 0, 0, 0],
                      header=Message(stamp=Message(sec=2, nanosec=0)))
        joints = joint_fields(msg)
        self.assertEqual(joints["left_wheel_radps"], 4)
        self.assertEqual(joints["caster_swivel_angle_rad"], 20)
        self.assertNotIn("effort", joints)
        msg.velocity = []
        with self.assertRaises(ValueError):
            joint_fields(msg)

    def test_bag_trace_interpolation_and_manifest_identity(self):
        command_hash = sha256(b"commands").hexdigest()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "trace.jsonl"
            mark = {"topic": "/janus/replay/event", "run_id": "M02-gazebo-cli",
                    "command_sha256": command_hash}

            def sample(topic, t, x, yaw):
                return {"topic": topic, "time_s": t, "x_m": x, "y_m": 0.0, "yaw_rad": yaw}

            rows = [
                {**mark, "event": "replay_start", "sim_time_s": 4.0},
                {"topic": "/clock", "time_s": 4.0}, {"topic": "/clock", "time_s": 4.06},
                sample("/odom", 4.0, 0.0, math.pi - 0.04),
                sample("/odom", 4.06, 0.12, -math.pi + 0.02),
                sample("/janus/ground_truth", 4.0, 0.01, math.pi - 0.03),
                sample("/janus/ground_truth", 4.06, 0.13, -math.pi + 0.03),
                {"topic": "/joint_states", "time_s": 4.0, "caster_swivel_angle_rad": 0.0},
                {"topic": "/joint_states", "time_s": 4.06, "caster_swivel_angle_rad": 0.06},
                {"topic": "/janus/cmd_vel/request", "bag_time_ns": 4_000_000_000,
                 "v_mps": 0.1, "yaw_rate_radps": 0.0},
                {"topic": "/janus/cmd_vel/request", "bag_time_ns": 4_025_000_000,
                 "v_mps": 0.2, "yaw_rate_radps": 0.0},
                {"topic": "/janus/cmd_vel/request", "bag_time_ns": 4_060_000_000,
                 "v_mps": 0.0, "yaw_rate_radps": 0.0},
                {**mark, "event": "replay_end", "sim_time_s": 4.06},
            ]
            path.write_text("\n".join(json.dumps(row) for row in rows))
            metrics, results = measure_trace(path, "M02-gazebo-cli", command_hash, 0.02)
            self.assertEqual(metrics["status"], "measured")
            self.assertAlmostEqual(metrics["odom_position_rmse_m"], 0.01)
            self.assertAlmostEqual(metrics["odom_yaw_rmse_rad"], 0.01)
            self.assertAlmostEqual(results[1]["time_s"], 0.02)
            self.assertAlmostEqual(results[1]["caster_swivel_angle_rad"], 0.02)
            self.assertGreater(results[1]["yaw_rad"], 3)
            self.assertEqual(results[1]["requested_v_mps"], 0.1)
            self.assertEqual(results[2]["requested_v_mps"], 0.2)
            self.assertEqual(metrics["command_overlap_samples"]["/cmd_vel"], 0)
            rejected, _ = measure_trace(path, "M02-gazebo-cli", "mismatch", 0.02)
            self.assertEqual(rejected["reason"], "command_hash_mismatch")


if __name__ == "__main__":
    unittest.main()
