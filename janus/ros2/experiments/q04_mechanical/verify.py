#!/usr/bin/env python3
"""Recompute Q04 optical and pose gates from retained external observations."""

import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path


def check(folder, original):
    fixture = json.loads((original / "fixture.json").read_text())
    execution = json.loads((folder / "execution.json").read_text())
    trace = defaultdict(list)
    with (folder / "telemetry.jsonl").open() as stream:
        for line in stream:
            data = json.loads(line)
            trace[data["source"]].append(data)
    events = {entry["event"]: entry["time_s"] for entry in trace["event"]}
    truth = trace["truth"]
    if not truth or not trace["qr"] or not trace["request"]:
        return {"status": "failed", "diagnostic": "missing native truth, QR observations or requests",
                "samples": {key: len(rows) for key, rows in trace.items()}}

    end_s = events.get("case_end", truth[-1]["time_s"])
    end = next((row for row in reversed(truth) if row["time_s"] <= end_s + .02), truth[-1])
    start = truth[0]
    start_x, start_y, _, _, _, start_yaw = fixture["spawn_xyz_rpy"]
    goal_x, goal_y, goal_yaw = fixture["dock_nominal_axle_xy_yaw"]
    marker_x, marker_y, *_ = fixture["marker_xyz_rpy"]
    optical_depth = ((marker_x - end["x_m"]) * math.cos(end["yaw_rad"])
                     + (marker_y - end["y_m"]) * math.sin(end["yaw_rad"]) - .25)
    pose_error = math.hypot(end["x_m"] - goal_x, end["y_m"] - goal_y)
    yaw_error = math.remainder(end["yaw_rad"] - goal_yaw, 2 * math.pi)
    terminal = [row for row in truth if end["time_s"] - .28 <= row["time_s"] <= end["time_s"]]
    pose_dwell = (bool(terminal) and terminal[-1]["time_s"] - terminal[0]["time_s"] >= .25
                  and all(math.hypot(row["x_m"] - goal_x, row["y_m"] - goal_y) <= .02
                          and abs(math.remainder(row["yaw_rad"] - goal_yaw, 2 * math.pi)) <= .05
                          and abs(row["v_mps"]) <= .02 and abs(row["yaw_rate_radps"]) <= .05
                          for row in terminal))
    qr = {row["capture_time_s"]: row for row in trace["qr"] if row["valid"]
          and row["id"] == "JANUS-DOCK-01" and row["observation_source"] == "decoded_image"}
    last_qr = next((row for row in reversed(list(qr.values())) if row["read_time_s"] <= end_s), None)
    qr_age = end_s - last_qr["capture_time_s"] if last_qr else None
    qr_error = last_qr["normalized_error"] if last_qr else None
    forward = [row for row in trace["request"] if row["v_mps"] > 1e-7]
    forwarding = [row for row in forward if row["phase"] == "optical_center"]
    staging = [row for row in forward if row["phase"] == "stage_translate"]
    invalid_forward = []
    for row in forward:
        frame = qr.get(row["capture_time_s"])
        if (frame is None or frame["read_time_s"] > row["time_s"]
                or not 0 <= row["time_s"] - frame["capture_time_s"] <= .20):
            invalid_forward.append(row)
    start_hash = hashlib.sha256((folder / "world.sdf").read_bytes()).hexdigest()
    original_hash = hashlib.sha256((original / "world.sdf").read_bytes()).hexdigest()
    staging_plan = trace["staging_goal"]
    first_positive = forwarding[0] if forwarding else None
    guard_after_hold = [row for row in trace["guard_output"] if row["time_s"] is not None
                        and events.get("terminal_hold", math.inf) + .15 <= row["time_s"] <= end_s]
    checks = {
        "unchanged_world_and_fixture": (start_hash == original_hash and
                                          json.loads((folder / "fixture.json").read_text()) == fixture),
        "unchanged_initial_position_yaw": (math.hypot(start["x_m"] - start_x, start["y_m"] - start_y) <= .002
                                          and abs(math.remainder(start["yaw_rad"] - start_yaw, 2 * math.pi)) <= .005),
        "bounded_run_completed": execution["completed"] and "case_end" in events,
        "one_publisher_per_runtime_topic": len(execution["publishers"]) == 6 and
                                           all(count == 1 for count in execution["publishers"].values()),
        "seven_distinct_fresh_camera_staging_measurements": (len(trace["marker_pose"]) == 7
            and len({row["capture_time_s"] for row in trace["marker_pose"]}) == 7 and len(staging_plan) == 1),
        "staging_and_optical_hold_are_distinct": bool(staging and forwarding) and len(forward) == len(staging) + len(forwarding),
        "valid_fresh_authentic_rendered_qr_for_all_forward_requests": bool(forward) and not invalid_forward,
        "staging_speed_within_100mmps": bool(staging) and max(row["v_mps"] for row in staging) <= .10,
        "inner_acquisition_and_outer_continuation_for_optical_approach": (bool(first_positive)
            and abs(first_positive["normalized_error"]) <= .02
            and all(abs(row["normalized_error"]) <= .04 for row in forwarding)),
        "independent_optical_depth_in_245_to_265mm": .245 <= optical_depth <= .265,
        "fresh_terminal_qr_within_two_percent": (last_qr is not None and 0 <= qr_age <= .20 and abs(qr_error) <= .02),
        "world_position_yaw_and_velocity_gates_sustained_250ms": pose_dwell,
        "physical_stationary_hold_has_zero_applied_twist": (bool(guard_after_hold)
            and all(abs(row["v_mps"]) <= 1e-8 and abs(row["yaw_rate_radps"]) <= 1e-8 for row in guard_after_hold)),
        "rendered_evidence_files": all((folder / (key + "-rendered.png")).is_file()
                                       for key in ("initial", "stage", "standoff", "terminal")),
    }
    initial = json.loads((original / "acceptance.json").read_text())["measurements"]
    return {
        "status": "passed" if all(checks.values()) else "failed",
        "scope": "supplemental simulated optics and axle pose on unchanged Q04 fixture; physical contact/BMS absent",
        "gates": checks,
        "measurements": {
            "terminal_optical_depth_m": optical_depth,
            "terminal_normalized_center_error": qr_error,
            "terminal_capture_age_s": qr_age,
            "terminal_position_error_m": pose_error,
            "terminal_yaw_error_rad": yaw_error,
            "terminal_world_pose": end,
            "terminal_s": end_s,
            "terminal_world_dwell_s": terminal[-1]["time_s"] - terminal[0]["time_s"] if terminal else None,
            "terminal_guard_output_count": len(guard_after_hold),
            "staging_forward_requests": len(staging),
            "optical_forward_requests": len(forwarding),
            "invalid_forward_requests": len(invalid_forward),
            "staging_observations": len(trace["marker_pose"]),
            "staging_plan": staging_plan,
            "original_Q04_terminal_position_error_m": initial["sim_nominal_dock_transform_error_m"],
            "original_Q04_terminal_yaw_error_rad": initial["sim_nominal_dock_yaw_error_rad"],
            "original_Q04_terminal_optical_depth_m": initial["independent_final_optical_depth_m"],
        },
        "world_sha256": start_hash,
    }


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("run", type=Path)
    cli.add_argument("original", type=Path)
    args = cli.parse_args()
    result = check(args.run, args.original)
    serialized = json.dumps(result, indent=2, allow_nan=False) + "\n"
    (args.run / "acceptance.json").write_text(serialized)
    print(serialized, end="")
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
