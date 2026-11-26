"""Measure offline ideal velocity replay error against the Python trajectory references."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from compare_replay import digest, pose_errors, read_columns


MOTION_CASES = (
    ("M04-python-nominal", "M04-gazebo-upstream-nominal-executed_velocity"),
    ("M05-python-cw", "M05-gazebo-upstream-cw-executed_velocity"),
    ("M06-python-ccw", "M06-gazebo-upstream-ccw-executed_velocity"),
)


def compare(python_root, offline_root):
    records = []
    for python_id, offline_id in MOTION_CASES:
        plant = python_root / python_id
        replay = offline_root / offline_id
        command_sha = digest(plant / "executed_velocity.csv")
        if command_sha != digest(replay / "commands.csv"):
            raise ValueError(f"Velocity commands differ in {offline_id}")
        actual = read_columns(plant / "results.csv", ("time_s", "x_m", "y_m", "yaw_rad"))
        reference = read_columns(plant / "results.csv", ("time_s", "ref_x_m", "ref_y_m", "ref_yaw_rad"))
        kinematic = read_columns(replay / "results.csv", ("time_s", "x_m", "y_m", "yaw_rad"))
        t = kinematic[:, 0]
        if abs(t[0] - actual[0, 0]) > 1e-9 or abs(t[-1] - actual[-1, 0]) > 1e-9:
            raise ValueError(f"Replay window differs in {offline_id}")

        def interpolate(series):
            return np.column_stack([np.interp(t, series[:, 0], values)
                                    for values in (series[:, 1], series[:, 2], np.unwrap(series[:, 3]))])

        planned = pose_errors(kinematic[:, 1:], interpolate(reference))
        dynamic = pose_errors(kinematic[:, 1:], interpolate(actual))
        records.append({
            "case_id": python_id[:3], "python_run_id": python_id, "offline_run_id": offline_id,
            "samples": len(t), "nominal_sample_period_s": float(np.median(np.diff(t))),
            "duration_s": float(t[-1]), "shared_velocity_sha256": command_sha,
            "planned_reference_position_rmse_m": planned["position_rmse_m"],
            "planned_reference_yaw_rmse_rad": planned["yaw_rmse_rad"],
            "planned_reference_endpoint_error_m": planned["endpoint_error_m"],
            "dynamic_python_position_rmse_m": dynamic["position_rmse_m"],
            "dynamic_python_yaw_rmse_rad": dynamic["yaw_rmse_rad"],
            "scope": "Open-loop ideal velocity integration on Mac. No Gazebo or ROS online tracking.",
        })
    return records


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("python_runs", type=Path)
    parser.add_argument("offline_runs", type=Path)
    parser.add_argument("output_csv", type=Path)
    args = parser.parse_args()
    records = compare(args.python_runs, args.offline_runs)
    with args.output_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=records[0].keys())
        writer.writeheader()
        writer.writerows(records)
    print(json.dumps(records, indent=2))
