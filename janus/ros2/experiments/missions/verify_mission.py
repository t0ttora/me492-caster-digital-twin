"""Verify JANUS mission acceptance from raw transport logs, independent of the live driver."""
import argparse
from bisect import bisect_right
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
from statistics import median
import xml.etree.ElementTree as ET

from contract import (
    BMS_FRESHNESS_S, CASES, CASTER_DWELL_S, CASTER_ERROR_LIMIT_RAD, CASTER_RATE_LIMIT_RADPS,
    CHARGE_WAIT_S, CONTACT_RELEASE_M, DOCK_CASES, DOCK_TARGET, DRIVE_RADIUS_M, GUIDES, LOW_SOC_LIMIT, NATIVE_CONTACT_LINKS,
    QR_FRESHNESS_S, QR_ID, QR_SIDE_M, QR_STANDOFF_M, SERVICE_TARGET, SPEED_LIMIT_MPS,
    STOP_DWELL_S, TRANSIT_TARGET, UNDOCK_TARGET_X, WAYPOINTS,
    dock_clearance, fresh_bms, in_position, pose_error, stopped, wrap, zero,
)

MAX_GAP_S = .10
EVENT_INSPECTION_S = .065
EXPECTED = {
    "Q15": "charge_timeout", "Q16": "virtual_charge_confirmed",
    "Q18": "lab_undock_completed", "O01": "route_completed",
    "O06": "route_completed", "O07": "virtual_charge_confirmed",
}


class Trace:
    def __init__(self, rows):
        self.data = defaultdict(list)
        self.events = defaultdict(list)
        self.rows = list(rows)
        for row in self.rows:
            self.data[row["source"]].append(row)
            if row["source"] == "event":
                self.events[row["event"]].append(row)
        self.timed = {
            source: sorted((r for r in self.data[source] if r.get("time_s") is not None),
                           key=lambda r: r["time_s"])
            for source in self.data
        }
        self.stamps = {source: [r["time_s"] for r in values] for source, values in self.timed.items()}
        self.start = self.when("case_start")

    def event(self, name):
        unique = self.events[name]
        return unique[0] if len(unique) == 1 else {}

    def when(self, name):
        return self.event(name).get("time_s")

    def ordered(self, names, *, equal=False):
        times = [self.when(name) for name in names]
        return all(t is not None for t in times) and all(
            a <= b if equal else a < b for a, b in zip(times, times[1:]))

    def since_start(self, timestamp):
        return timestamp - self.start if timestamp is not None and self.start is not None else None

    def nearest(self, source, stamp, tolerance=EVENT_INSPECTION_S):
        if stamp is None or not self.stamps.get(source):
            return {}
        times = self.stamps[source]
        i = bisect_right(times, stamp)
        options = self.timed[source][max(0, i-1):i+1]
        row = min(options, key=lambda r: abs(r["time_s"]-stamp))
        return row if abs(row["time_s"]-stamp) <= tolerance else {}

    def slice(self, source, start, end):
        if start is None or end is None or end < start:
            return []
        return [r for r in self.timed.get(source, ()) if start <= r["time_s"] <= end]

    def observed(self, source, start, end, predicate):
        rows = self.slice(source, start, end)
        if len(rows) < 2:
            return False
        coverage = [start, *(r["time_s"] for r in rows), end]
        return max(b-a for a, b in zip(coverage, coverage[1:])) <= MAX_GAP_S and all(map(predicate, rows))

    def state_hold(self, start, end, goal=None):
        def body_ok(body):
            odom = self.nearest("odom", body["time_s"])
            joints = self.nearest("joint", body["time_s"])
            return bool(odom and joints and stopped(body, odom, joints)
                        and (goal is None or in_position(body, goal)))

        return all(self.observed(kind, start, end, predicate)
                   for kind, predicate in (("truth", body_ok), ("odom", lambda r: True),
                                           ("joint", lambda r: True), ("request", zero), ("guard_output", zero)))

    def pose_metrics(self, stamp, goal):
        body = self.nearest("truth", stamp)
        if not body:
            return {"position_error_m": None, "yaw_error_rad": None, "world_pose": None}
        position, yaw = pose_error(body, goal)
        return {"position_error_m": position, "yaw_error_rad": yaw,
                "world_pose": [body[k] for k in ("x_m", "y_m", "yaw_rad")]}

    def signal_at(self, topic, event, value, after):
        target = self.event(event)
        if not target or after is None or "sequence" not in target:
            return {}
        matches = [r for r in self.data[topic] if r["sequence"] == target["sequence"]]
        if len(matches) != 1:
            return {}
        row = matches[0]
        if (row["origin"] != "virtual_station_fixture" or row["value"] != value
                or row["read_time_s"] is None or not after < row["capture_time_s"] <= row["read_time_s"]
                or not row["read_time_s"] <= target["time_s"]
                or not fresh_bms(row["capture_time_s"], target["time_s"], after)):
            return {}
        return row


def accepted_qr(row):
    return (row["valid"] and row["id"] == QR_ID and row["observation_source"] == "decoded_image"
            and row["frame_id"] == "camera_optical_frame" and len(row["corners_px"]) == 4
            and row["capture_time_s"] <= row["arrival_time_s"] <= row["publish_time_s"]
            and row["publish_time_s"] - row["capture_time_s"] <= QR_FRESHNESS_S)


def checked_center_dwell(trace):
    newest = None
    started = None
    pass_count = 0
    permitted = False
    for row in trace.rows:
        if row["source"] == "qr":
            newest = row
        if row["source"] != "request" or row["phase"] != "approach":
            continue
        usable = bool(newest and accepted_qr(newest)
                      and 0 <= row["time_s"]-newest["capture_time_s"] <= QR_FRESHNESS_S)
        if not usable:
            started, permitted = None, False
        else:
            error = abs(newest["normalized_error"])
            if error > .04:
                permitted = False
            if not permitted and error > .02:
                started = None
            elif not permitted:
                if started is None:
                    started = newest["arrival_time_s"]
                permitted = newest["arrival_time_s"]-started >= .40
        if row["v_mps"] > 0:
            if not permitted or not usable:
                return False
            pass_count += 1
    return pass_count > 0


def qr_approach(trace, output):
    checks, metrics = {}, {}
    valid = {row["capture_time_s"]: row for row in trace.data["qr"] if accepted_qr(row)}
    advances = [row for row in trace.data["request"]
                if row["phase"] == "approach" and row["v_mps"] > 0]
    matched_images = [
        row["file"] for row in trace.data["image"]
        if row["capture_time_s"] in valid and row["frame_id"] == "camera_optical_frame"
        and (output / row["file"]).is_file()
    ]

    def permitted(request):
        frame = valid.get(request.get("capture_time_s"))
        return bool(frame and 0 <= request["time_s"]-frame["capture_time_s"] <= QR_FRESHNESS_S
                    and abs(frame["normalized_error"]) <= .04
                    and frame["normalized_error"] * request["yaw_rate_radps"] <= 1e-12)

    start = trace.nearest("truth", trace.when("dock_started"))
    alignment = trace.nearest("truth", trace.when("aligned"))
    optical = trace.event("optical_stop")
    frame = valid.get(optical.get("capture_time_s"))
    depth = None
    if trace.data["intrinsics"] and frame:
        c0, c1, c2, c3 = frame["corners_px"]
        height = .5*(math.dist(c0, c3)+math.dist(c1, c2))
        depth = trace.data["intrinsics"][0]["matrix"][0] * QR_SIDE_M / height
    advance_beg = trace.when("qr_advance")
    checks["live_decoded_qr_drives_forward"] = bool(advances and matched_images and all(map(permitted, advances)))
    checks["fresh_qr_center_dwell_reconstructed"] = checked_center_dwell(trace)
    checks["optical_stop_from_logged_corners"] = bool(
        depth is not None and depth <= QR_STANDOFF_M
        and math.isclose(depth, optical["estimated_depth_m"], rel_tol=1e-9))
    checks["native_forward_motion_after_camera_acquisition"] = bool(
        trace.ordered(("dock_started", "qr_acquired", "qr_advance", "optical_stop"), equal=True)
        and start and alignment and alignment["x_m"]-start["x_m"] > .10
        and any(r["v_mps"] > .02 for r in trace.slice("truth", advance_beg, trace.when("optical_stop"))))
    metrics.update(valid_qr_frames=len(valid), forward_qr_requests=len(advances),
                   matched_decoded_pngs=matched_images,
                   optical_stop_depth_m=depth,
                   qr_progress_m=alignment["x_m"]-start["x_m"] if start and alignment else None)
    return checks, metrics


def charge(trace, case):
    checks, metrics = {}, {}
    aligned, rest = trace.when("aligned"), trace.when("zero_motion")
    wait, confirm, success = (trace.when(name) for name in ("wait_for_charge", "charge_confirmed", "dock_success"))
    end = trace.when("reverse_enabled") if case == "Q18" else trace.when("terminal")
    aligned_body = trace.nearest("truth", aligned)
    checks["separated_world_alignment_stop_and_virtual_contact"] = bool(
        trace.ordered(("aligned", "zero_motion", "contact")) and wait == trace.when("contact")
        and bool(trace.signal_at("contact", "contact", True, rest))
        and aligned_body and in_position(aligned_body, DOCK_TARGET))
    checks["continuous_odometry_encoders_and_zero_commands"] = bool(
        rest is not None and trace.state_hold(rest-STOP_DWELL_S, end, DOCK_TARGET))
    metrics.update(trace.pose_metrics(rest, DOCK_TARGET))
    metrics.update(wait_s=(confirm or trace.when("charge_timeout")) - wait
                   if wait is not None and (confirm or trace.when("charge_timeout")) is not None else None)
    if rest is not None and end is not None:
        hold = trace.slice("truth", rest, end)
        wheels = trace.slice("joint", rest, end)
        metrics.update(hold_max_world_speed_mps=max((abs(r["v_mps"]) for r in hold), default=None),
                       hold_max_world_yaw_rate_radps=max((abs(r["yaw_rate_radps"]) for r in hold), default=None),
                       hold_max_drive_rim_speed_mps=max((DRIVE_RADIUS_M*abs(r[side+"_wheel_radps"])
                                                        for r in wheels for side in ("left", "right")), default=None))
    if case == "Q15":
        timeout = trace.when("charge_timeout")
        checks["full_wait_budget_then_timeout"] = bool(
            wait is not None and timeout is not None
            and CHARGE_WAIT_S <= timeout-wait <= CHARGE_WAIT_S+MAX_GAP_S)
        checks["no_charge_or_success_in_trace"] = (not trace.events["charge_confirmed"]
                                                    and not trace.events["dock_success"]
                                                    and not any(r["value"] for r in trace.data["bms"]))
        return checks, metrics
    ack = trace.signal_at("bms", "charge_confirmed", True, wait)
    checks["fresh_bms_ack_before_success"] = bool(
        ack and wait < confirm <= success and confirm-wait <= CHARGE_WAIT_S
        and trace.event("dock_success").get("sequence") == ack["sequence"])
    metrics["bms_ack_age_s"] = confirm - ack["capture_time_s"] if ack else None
    early = [r for r in trace.data["bms"] if r["stimulus"] == "precontact_charge"]
    expired = [r for r in trace.data["bms"] if r["stimulus"] == "expired_charge"]
    checks["precontact_and_stale_ack_do_not_complete"] = bool(
        ack and len(early) == len(expired) == 1 and early[0]["value"] is True and expired[0]["value"] is True
        and early[0]["read_time_s"] < wait < expired[0]["capture_time_s"] <= expired[0]["read_time_s"]
        < ack["capture_time_s"] and expired[0]["read_time_s"]-expired[0]["capture_time_s"] > BMS_FRESHNESS_S
        and confirm > expired[0]["read_time_s"])
    return checks, metrics


def caster_recovery(rows):
    start = None
    previous = None
    for timestamp, angle, rate, speed in rows:
        if (abs(wrap(angle-math.pi)) <= CASTER_ERROR_LIMIT_RAD and abs(rate) <= CASTER_RATE_LIMIT_RADPS
                and speed < -SPEED_LIMIT_MPS):
            if start is None or previous is None or timestamp-previous > MAX_GAP_S:
                start = timestamp
            if timestamp-start >= CASTER_DWELL_S:
                return timestamp
        else:
            start = None
        previous = timestamp
    return None


def world_guide_boxes(output):
    world = ET.parse(output / "world.sdf").find("world")
    boxes = {}
    for name in GUIDES:
        model = world.find(f"model[@name='{name}']")
        if model is None or model.findtext("static") != "true":
            raise ValueError(f"{name} is absent or mobile in the executed SDF")
        x, y, _, roll, pitch, yaw = map(float, model.findtext("pose").split())
        if any((roll, pitch, yaw)):
            raise ValueError(f"{name} is not an axis-aligned fixture")
        width, depth, _ = map(float, model.findtext("link/collision/geometry/box/size").split())
        boxes[name] = (x, y, width, depth)
    return boxes


def undock(trace, output):
    checks, metrics = {}, {}
    cut_request, cut_ack, reverse, end = (trace.when(name) for name in (
        "charge_cut_request", "charge_cut_confirmed", "reverse_enabled", "exit_stopped"))
    req = [r for r in trace.data["request"] if r["phase"] == "reverse" and r["v_mps"] < 0.]
    enabled = req[0]["time_s"] if req else None
    reverse_out = [r for r in trace.slice("guard_output", reverse, end) if r["v_mps"] < 0.]
    checks["explicit_lab_authority_cut_then_measured_reverse"] = bool(
        trace.ordered(("dock_success", "undock_authorized", "charge_cut_request",
                       "charge_cut_confirmed", "reverse_enabled"), equal=True)
        and trace.when("dock_success") < trace.when("undock_authorized") < cut_ack < reverse
        and trace.signal_at("bms", "charge_cut_confirmed", False, cut_request)
        and enabled is not None and reverse < enabled and reverse_out
        and enabled <= reverse_out[0]["time_s"])
    released = trace.when("contact_released")
    release_row = trace.nearest("truth", released)
    checks["virtual_contact_released_after_gazebo_displacement"] = bool(
        trace.signal_at("contact", "contact_released", False, reverse)
        and release_row and release_row["x_m"] <= DOCK_TARGET[0]-CONTACT_RELEASE_M)
    truth = trace.slice("truth", reverse, end)
    joints = [trace.nearest("joint", row["time_s"]) for row in truth]
    paired = [(row, j) for row, j in zip(truth, joints) if j]
    guides = world_guide_boxes(output)
    clearance = [dock_clearance(row, j, guides) for row, j in paired]
    min_clearance = min(clearance, default=None)
    checks["positive_native_planar_clearance"] = bool(
        len(truth) >= 2 and len(paired) == len(truth) and min_clearance is not None and min_clearance > 0.
        and trace.observed("truth", reverse, end, lambda _: True))
    native_distance = truth[0]["x_m"]-truth[-1]["x_m"] if len(truth) >= 2 else None
    checks["measured_native_reverse_and_exit_stop"] = bool(
        native_distance is not None and native_distance > .50
        and any(row["v_mps"] < -.05 and j["left_wheel_radps"] < -.5 and j["right_wheel_radps"] < -.5
                for row, j in paired)
        and trace.state_hold(end-STOP_DWELL_S, end, (UNDOCK_TARGET_X, 0., 0.)))
    recovery = caster_recovery((row["time_s"], j["caster_swivel_angle_rad"], j["caster_swivel_radps"], row["v_mps"])
                               for row, j in paired)
    checks["native_caster_recovery_while_reversing"] = recovery is not None
    metrics.update(charge_cut_to_first_reverse_request_s=enabled-cut_ack if enabled is not None and cut_ack is not None else None,
                   charge_cut_to_first_guarded_reverse_s=reverse_out[0]["time_s"]-cut_ack
                   if cut_ack is not None and reverse_out else None,
                   minimum_planar_clearance_m=min_clearance, clearance_samples=len(clearance),
                   native_backout_distance_m=native_distance,
                   caster_recovery_after_reverse_s=recovery-reverse if recovery is not None else None,
                   terminal_caster_error_to_reverse_rad=abs(wrap(joints[-1]["caster_swivel_angle_rad"]-math.pi))
                   if joints and joints[-1] else None,
                   maximum_abs_swivel_rate_radps=max((abs(j["caster_swivel_radps"]) for _, j in paired), default=None))
    metrics.update({"exit_" + name: value for name, value in
                    trace.pose_metrics(end, (UNDOCK_TARGET_X, 0., 0.)).items()})
    return checks, metrics


def route(trace, case, output):
    expected = WAYPOINTS if case == "O01" else (("service-park", SERVICE_TARGET),)
    events = {name: trace.events[name] for name in ("mission_reset", "goal_started", "arrived")}
    checks = {"exactly_ordered_distinct_mission_ids": all(
        [r["mission_id"] for r in rows] == [mission for mission, _ in expected] for rows in events.values())}
    all_stopped = True
    all_dispatched = True
    arrival_metrics = []
    previous_arrival = -math.inf
    for i, (mission, goal) in enumerate(expected):
        if any(len(rows) <= i for rows in events.values()):
            all_dispatched = all_stopped = False
            continue
        reset, start, arrived = (events[name][i] for name in ("mission_reset", "goal_started", "arrived"))
        a, b, c = (row["time_s"] for row in (reset, start, arrived))
        all_dispatched &= (previous_arrival < a <= b < c and c-b >= STOP_DWELL_S
                           and reset["previous_goal_cleared"] and reset["prior_stop_dwell_s"] == 0.
                           and reset["charge_state"] == "idle"
                           and tuple(start["target_xyz"]) == tuple(arrived["target_xyz"]) == goal)
        all_stopped &= trace.state_hold(c-STOP_DWELL_S, c, goal)
        previous_arrival = c
        arrival_metrics.append({"mission_id": mission, "target_xyz": goal, "time_s": trace.since_start(c),
                                **trace.pose_metrics(c, goal)})
    checks["per_goal_reset_and_independent_stopped_pose_dwell"] = all_dispatched and all_stopped
    metrics = {"arrivals": arrival_metrics}
    if case == "O01":
        signs = {"A-in": 0, "B": 1, "A-out": -1}
        output_requests = [row for row in trace.data["request"] if row["phase"] == "route"]
        checks["route_requests_identify_current_goal_and_both_directions"] = bool(
            output_requests and all(row.get("mission_id") in signs
                                    and row["v_mps"] * signs[row["mission_id"]] >= 0.
                                    for row in output_requests)
            and any(row["v_mps"] > .05 and row["mission_id"] == "B" for row in output_requests)
            and any(row["v_mps"] < -.05 and row["mission_id"] == "A-out" for row in output_requests))
    if case == "O06":
        model = ET.parse(output / "loaded-arche.sdf")
        joint = model.find("./model/joint[@name='janus_lab_payload_fixed']")
        mass = model.findtext("./model/link[@name='janus_lab_payload']/inertial/mass")
        spawned = ET.parse(output / "world.sdf").find("./world/model[@name='arche']")
        spawned_payload_mass = spawned.findtext("link[@name='janus_lab_payload']/inertial/mass") if spawned is not None else None
        checks["four_kg_rigid_payload_in_executed_sdf"] = bool(
            mass is not None and float(mass) == 4. and mass == spawned_payload_mass
            and joint is not None and joint.get("type") == "fixed"
            and joint.findtext("parent") == "base_link" and joint.findtext("child") == "janus_lab_payload")
        checks["measured_loaded_transit"] = bool(
            trace.data["truth"] and trace.data["truth"][-1]["x_m"]-trace.data["truth"][0]["x_m"] > .45)
        settled_start = previous_arrival-STOP_DWELL_S
        contacts = {
            wheel: [(row["time_s"], pair[f"force_z_body_{side}_N"])
                    for row in trace.data["floor_contact"] if row["wheel"] == wheel
                    for pair in row["pairs"] if pair["points"] > 0
                    for side, other in ((1, 2), (2, 1))
                    if "ground" in pair[f"collision_{other}"] and wheel in pair[f"collision_{side}"]]
            for wheel in NATIVE_CONTACT_LINKS
        }
        support = {wheel: [force for stamp, force in samples if settled_start <= stamp <= previous_arrival]
                   for wheel, samples in contacts.items()}
        checks["native_ground_contact_measured_on_each_wheel"] = all(contacts.values()) and all(support.values())
        support_medians = {wheel: median(forces) if forces else None for wheel, forces in support.items()}
        metrics["native_floor_contact_sample_counts"] = {wheel: len(samples) for wheel, samples in contacts.items()}
        metrics["settled_median_vertical_wheel_support_n"] = support_medians
        metrics["settled_sum_of_median_wheel_support_n"] = sum(support_medians.values()) if all(support.values()) else None

    return checks, metrics


def low_soc(trace):
    arrival, started, handoff, stopped_at, docked = (trace.when(name) for name in (
        "soc_below_threshold", "transit_started", "transit_preempted", "transit_stopped", "dock_started"))
    nominal = [r for r in trace.data["soc"] if r["stimulus"] == "nominal_transit_soc"]
    sequence = trace.event("soc_below_threshold").get("sequence")
    low = [r for r in trace.data["soc"] if r["sequence"] == sequence]
    before = trace.slice("truth", started, handoff)
    checks = {
        "active_transit_preempted_by_fresh_synthetic_soc": bool(
            trace.ordered(("transit_started", "soc_below_threshold", "transit_preempted",
                           "transit_stopped", "dock_started", "charge_confirmed", "dock_success"), equal=True)
            and started < arrival <= handoff < stopped_at <= docked
            and len(nominal) == len(low) == 1
            and all(r["origin"] == "virtual_station_fixture" for r in (*nominal, *low))
            and nominal[0]["value"] > LOW_SOC_LIMIT >= low[0]["value"]
            and started <= nominal[0]["capture_time_s"] < low[0]["capture_time_s"] <= low[0]["read_time_s"] <= arrival
            and arrival-low[0]["capture_time_s"] <= BMS_FRESHNESS_S
            and tuple(trace.event("transit_started")["target_xyz"]) == TRANSIT_TARGET
            and before and before[0]["x_m"]-before[-1]["x_m"] > .08
            and before[-1]["x_m"] > TRANSIT_TARGET[0]+.1
            and any(row["v_mps"] < -.05 for row in before)),
        "native_stop_dwell_between_mission_priorities": bool(
            stopped_at is not None and trace.state_hold(stopped_at-STOP_DWELL_S, stopped_at)),
    }
    return checks, {"low_soc_fraction": low[0]["value"] if low else None,
                    "transit_distance_at_preemption_m": before[0]["x_m"]-before[-1]["x_m"] if before else None}


def assess(case, output):
    with (output / "telemetry.jsonl").open() as stream:
        trace = Trace(json.loads(line) for line in stream)
    ending = trace.when("terminal")
    final_clock = trace.stamps.get("clock", [])
    checks = {
        "expected_outcome_and_post_terminal_zero": bool(
            ending is not None and final_clock and final_clock[-1]-ending >= .32
            and trace.event("terminal").get("outcome") == EXPECTED[case]
            and all(trace.observed(source, ending, final_clock[-1], zero) for source in ("request", "guard_output"))),
        "native_ros_clock_world_odom_and_encoders": all(
            len(trace.data[s]) > 10 and all(a["time_s"] < b["time_s"]
                                           for a, b in zip(trace.data[s], trace.data[s][1:]))
            for s in ("clock", "truth", "odom", "joint")),
    }
    metrics = {}
    groups = []
    if case in DOCK_CASES:
        groups.extend([qr_approach(trace, output), charge(trace, case)])
    if case == "Q18":
        groups.append(undock(trace, output))
    elif case in {"O01", "O06"}:
        groups.append(route(trace, case, output))
    elif case == "O07":
        groups.append(low_soc(trace))
    for more_checks, more_metrics in groups:
        checks.update(more_checks)
        metrics.update(more_metrics)
    return {
        "case_id": case,
        "status": "lab_simulation_pass" if all(checks.values()) else "partial_or_failed",
        "checks": checks, "measurements": metrics,
        "event_times_s_relative": {name: [trace.since_start(r["time_s"]) for r in events]
                                   for name, events in trace.events.items() if events},
        "samples": dict(Counter({kind: len(rows) for kind, rows in trace.data.items() if rows})),
        "scope": "native Gazebo/ROS. Station contact, BMS, SoC, and authorization signals are laboratory fixtures.",
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case", choices=CASES)
    parser.add_argument("output", type=Path)
    opts = parser.parse_args()
    result = assess(opts.case, opts.output)
    rendered = json.dumps(result, indent=2, allow_nan=False) + "\n"
    (opts.output / "verification.json").write_text(rendered)
    print(rendered, end="")
    raise SystemExit(0 if result["status"] == "lab_simulation_pass" else 2)
