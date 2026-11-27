"""Analytic and fail-closed tests for the mission log verifier."""
import math
import unittest

from contract import (
    DOCK_TARGET, GUIDES, dock_clearance, fresh_bms, rectangle,
    rectangle_clearance, stopped,
)
from verify_mission import Trace, caster_recovery, charge


def event(name, time, **kwargs):
    return {"source": "event", "event": name, "time_s": time, **kwargs}


def signal(topic, sequence, value, capture, read, stimulus=""):
    return {"source": topic, "sequence": sequence, "value": value,
            "origin": "virtual_station_fixture", "capture_time_s": capture,
            "read_time_s": read, "stimulus": stimulus}


def motion(time):
    stationary = {"time_s": time, "v_mps": 0., "yaw_rate_radps": 0.}
    return [
        dict(stationary, source="truth", x_m=DOCK_TARGET[0], y_m=0., yaw_rad=0.),
        dict(stationary, source="odom"),
        {"source": "joint", "time_s": time, "left_wheel_radps": 0., "right_wheel_radps": 0.},
        dict(stationary, source="request"),
        dict(stationary, source="guard_output"),
    ]


def charge_rows(ack):
    end = 1.50 if ack else 2.50
    rows = [
        event("case_start", 0.),
        event("aligned", .60),
        event("zero_motion", .85),
        signal("contact", 1, True, .95, .975),
        event("contact", 1., sequence=1),
        event("wait_for_charge", 1.),
        event("terminal", end),
    ]
    if ack:
        rows += [
            signal("bms", 2, True, .20, .21, "precontact_charge"),
            signal("bms", 3, True, 1.03, 1.36, "expired_charge"),
            signal("bms", 4, True, 1.47, 1.48, "fresh_charge"),
            event("charge_confirmed", 1.50, sequence=4),
            event("dock_success", 1.50, sequence=4),
        ]
    else:
        rows.append(event("charge_timeout", end))
    for i in range(10, int(end*40)+2):
        rows.extend(motion(i/40))
    return rows


class EvidenceTests(unittest.TestCase):
    def test_fresh_ack_cannot_be_early_stale_or_future(self):
        self.assertTrue(fresh_bms(1.3, 1.35, 1.))
        self.assertFalse(fresh_bms(1.0, 1.1, 1.))
        self.assertFalse(fresh_bms(1.1, 1.35, 1.))
        self.assertFalse(fresh_bms(1.4, 1.35, 1.))

    def test_encoder_detects_motion_with_zero_command_and_zero_odometry(self):
        truth, odom, joints, _, _ = motion(0.)
        joints["left_wheel_radps"] = .4
        self.assertFalse(stopped(truth, odom, joints))

    def test_clearance_touch_separation_diagonal_and_rotation(self):
        origin = rectangle(0., 0., 2., 2.)
        self.assertAlmostEqual(rectangle_clearance(origin, rectangle(2., 0., 2., 2.)), 0.)
        self.assertAlmostEqual(rectangle_clearance(origin, rectangle(1.5, 0., 2., 2.)), -.5)
        self.assertAlmostEqual(rectangle_clearance(origin, rectangle(3., 3., 2., 2.)), math.sqrt(2))
        self.assertAlmostEqual(
            rectangle_clearance(origin, rectangle(3., 0., 2., 2., math.pi/4)), 2-math.sqrt(2))
        self.assertAlmostEqual(
            dock_clearance({"x_m": DOCK_TARGET[0], "y_m": 0., "yaw_rad": 0.},
                           {"caster_swivel_angle_rad": 0.}, GUIDES), .085)

    def test_recovery_needs_moving_window_and_clock_coverage(self):
        still = [(i/40, math.pi, .03, 0.) for i in range(20)]
        reversing = [(i/40, -math.pi+.01, .03, -.10) for i in range(20)]
        self.assertIsNone(caster_recovery(still))
        self.assertIsNone(caster_recovery(reversing[:4] + reversing[14:]))
        self.assertAlmostEqual(caster_recovery(reversing), .25)

    def test_timeout_rejects_truncated_wait_and_nonzero_guard(self):
        rows = charge_rows(ack=False)
        checks, _ = charge(Trace(rows), "Q15")
        self.assertTrue(all(checks.values()), checks)
        next(r for r in rows if r.get("event") == "charge_timeout")["time_s"] = 2.10
        next(r for r in rows if r["source"] == "guard_output" and r["time_s"] == 1.25)["v_mps"] = .01
        checks, _ = charge(Trace(rows), "Q15")
        self.assertFalse(checks["full_wait_budget_then_timeout"])
        self.assertFalse(checks["continuous_odometry_encoders_and_zero_commands"])

    def test_success_rejects_claim_without_matching_fresh_packet(self):
        rows = charge_rows(ack=True)
        checks, _ = charge(Trace(rows), "Q16")
        self.assertTrue(all(checks.values()), checks)
        for row in rows:
            if row.get("event") in {"charge_confirmed", "dock_success"}:
                row["sequence"] = 3
        checks, _ = charge(Trace(rows), "Q16")
        self.assertFalse(checks["fresh_bms_ack_before_success"])

    def test_success_rejects_missing_native_measurements(self):
        rows = [r for r in charge_rows(ack=True)
                if r["source"] != "joint" or not 1.0 <= r["time_s"] <= 1.4]
        checks, _ = charge(Trace(rows), "Q16")
        self.assertFalse(checks["continuous_odometry_encoders_and_zero_commands"])


if __name__ == "__main__":
    unittest.main()
