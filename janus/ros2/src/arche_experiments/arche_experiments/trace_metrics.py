"""Read a bag export, align using replay markers and compare simultaneous states."""
from collections import Counter
import json
import math

import numpy as np


def measure_trace(path, run_id, command_hash, grid_s):
    command_topics = ("/janus/cmd_vel/request", "/cmd_vel")
    streams = {topic: [] for topic in ("/odom", "/janus/ground_truth", "/joint_states", "/clock",
                                        "/janus/replay/event", *command_topics)}
    counts, errors = Counter(), Counter()
    with path.open() as stream:
        for line in stream:
            sample = json.loads(line)
            topic = sample["topic"]
            counts[topic] += 1
            if "error" in sample:
                errors[topic] += 1
            elif topic in streams:
                streams[topic].append(sample)
    events = [row for row in streams["/janus/replay/event"] if row.get("run_id") == run_id]
    start = [row for row in events if row.get("event") == "replay_start"]
    end = [row for row in events if row.get("event") == "replay_end"]
    metrics = {"topic_counts": dict(counts), "decode_errors": dict(errors),
               "status": "blocked", "grid_s": grid_s}
    if len(start) != 1 or len(end) != 1 or len(events) != 2:
        return {**metrics, "reason": "missing_or_incomplete_replay_markers"}, []
    if any(row["command_sha256"] != command_hash for row in (start[0], end[0])):
        return {**metrics, "reason": "command_hash_mismatch"}, []
    start_s, end_s = start[0]["sim_time_s"], end[0]["sim_time_s"]
    if end_s <= start_s:
        return {**metrics, "reason": "invalid_run_duration"}, []
    clock = [row["time_s"] for row in streams["/clock"]]
    if len(clock) < 2 or any(b < a for a, b in zip(clock, clock[1:])):
        return {**metrics, "reason": "missing_or_reset_clock"}, []
    groups = [streams[topic] for topic in ("/odom", "/janus/ground_truth", "/joint_states")]
    if any(len(group) < 2 for group in groups):
        return {**metrics, "reason": "missing_state_stream"}, []
    time = [np.array([row["time_s"] for row in group]) for group in groups]
    if any(np.any(np.diff(t) <= 0) for t in time):
        return {**metrics, "reason": "nonmonotonic_state_stream"}, []
    lower = max(start_s, *(t[0] for t in time))
    upper = min(end_s, *(t[-1] for t in time))
    grid = start_s + np.arange(math.ceil((lower - start_s) / grid_s),
                               math.floor((upper - start_s) / grid_s) + 1) * grid_s
    if len(grid) < 2:
        return {**metrics, "reason": "no_overlapping_state"}, []
    columns = {}
    for prefix, group, stamps in zip(("odom_", "", ""), groups, time):
        for key in group[0]:
            if key in ("topic", "bag_time_ns", "time_s"):
                continue
            values = np.array([row[key] for row in group])
            if key.endswith(("yaw_rad", "_angle_rad")):
                values = np.unwrap(values)
            columns[prefix + key] = np.interp(grid, stamps, values)
    xy = np.hypot(columns["x_m"] - columns["odom_x_m"], columns["y_m"] - columns["odom_y_m"])
    angle = columns["yaw_rad"] - columns["odom_yaw_rad"]
    angle = np.arctan2(np.sin(angle), np.cos(angle))
    rows = [{"time_s": float(t - start_s), **{k: float(values[i]) for k, values in columns.items()}}
            for i, t in enumerate(grid)]
    coverage = {}
    for topic, prefix in zip(command_topics, ("requested_", "guard_output_")):
        commands = streams[topic]
        stamps = np.array([row["bag_time_ns"] / 1e9 for row in commands])
        if not len(stamps) or np.any(np.diff(stamps) < 0):
            coverage[topic] = 0
            continue
        indexes = np.searchsorted(stamps, grid, side="right") - 1
        valid = (indexes >= 0) & (grid <= stamps[-1])
        coverage[topic] = int(valid.sum())
        for row, index, covered in zip(rows, indexes, valid):
            for key in ("v_mps", "yaw_rate_radps"):
                row[prefix + key] = commands[index][key] if covered else None
    return {**metrics, "status": "measured", "overlap_samples": len(grid),
            "observed_start_s": float(grid[0] - start_s),
            "observed_end_s": float(grid[-1] - start_s),
            "state_max_gap_s": {topic: float(np.max(np.diff(stamps))) for topic, stamps in
                                zip(("/odom", "/janus/ground_truth", "/joint_states"), time)},
            "odom_position_rmse_m": float(np.sqrt(np.mean(xy ** 2))),
            "odom_yaw_rmse_rad": float(np.sqrt(np.mean(angle ** 2))),
            "ground_truth_path_length_m": float(np.hypot(np.diff(columns["x_m"]), np.diff(columns["y_m"])).sum()),
            "ground_truth_max_displacement_m": float(np.hypot(
                columns["x_m"] - columns["x_m"][0], columns["y_m"] - columns["y_m"][0]).max()),
            "command_overlap_samples": coverage,
            "command_time_source": "rosbag2 record --use-sim-time receive time",
            "applied_axle_effort": None, "caster_contact_force": None}, rows
