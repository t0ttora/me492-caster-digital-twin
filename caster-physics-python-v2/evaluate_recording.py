"""Replay logged axle torques and compare M0/M2 with an independent axle pose."""
import argparse
import csv
from dataclasses import replace
import hashlib
import json
from pathlib import Path

import numpy as np

from twin import (State, Torques, load_parameters, validate_parameters, integrate,
                  wheel_odometry, pose_errors, geometry_at, verify_v1)
from run_study import new_output_directory, write_json, write_csv


def evaluate(csv_path, metadata_path):
    metadata = json.loads(Path(metadata_path).read_text())
    required = ("run_id", "session_id", "split", "data_kind", "surface_id", "robot_revision",
                "pose_source", "initial_state", "parameter_provenance", "torque_source")
    if any(key not in metadata or not metadata[key] for key in required):
        raise ValueError("Missing recording provenance fields")
    if metadata["data_kind"] not in {"measured", "synthetic"} or metadata["split"] not in {"development", "confirmation"}:
        raise ValueError("Unknown data kind or split")
    if metadata.get("pose_frame") != "drive_axle_midpoint_world_SI" or metadata.get("torque_units") != "axle_Nm":
        raise ValueError("Use SI world-frame axle poses and axle torques, not volts or speed commands")
    if metadata["pose_source"] in {"odometry", "wheel_odometry", "encoders"}:
        raise ValueError("The pose reference must be independent of wheel odometry")
    base = load_parameters()
    overrides = metadata.get("parameter_overrides", {})
    if set(overrides) - {"geometry", "dynamics"}:
        raise ValueError("Unknown parameter override group")
    parameters = replace(base, geometry=replace(base.geometry, **overrides.get("geometry", {})),
                         dynamics=replace(base.dynamics, **overrides.get("dynamics", {})))
    validate_parameters(parameters)
    with Path(csv_path).open(newline="", encoding="utf-8-sig") as file:
        rows = list(csv.DictReader(file))
    columns = ("time_s", "left_drive_torque_Nm", "right_drive_torque_Nm", "left_wheel_rate_radps",
               "right_wheel_rate_radps", "reference_x_m", "reference_y_m", "reference_yaw_rad")
    try:
        values = np.array([[float(row[key]) for key in columns] for row in rows])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("Missing or invalid recording columns") from error
    if values.ndim != 2 or len(values) < 2 or not np.all(np.isfinite(values)):
        raise ValueError("Recording needs at least two finite rows")
    times = values[:, 0]
    d = parameters.dynamics
    if abs(times[0]) > 1e-12 or not np.allclose(np.diff(times), d.command_sample_dt_s, atol=1e-10, rtol=0):
        raise ValueError("Recording must start at zero with one row per command interval; gaps are rejected")
    if abs(d.result_sample_dt_s - d.command_sample_dt_s) > 1e-12:
        raise ValueError("Replay requires equal command and result intervals")
    start = State.from_vector(metadata["initial_state"])
    rotation = geometry_at(start, parameters).body_rotation
    axle = np.array([start.body_x_m, start.body_y_m]) - rotation @ np.array(d.body_com_m[:2])
    if (not np.allclose([*axle, start.body_yaw_rad], values[0, 5:8], atol=1e-8, rtol=0)
            or not np.allclose([start.left_wheel_rate_radps, start.right_wheel_rate_radps], values[0, 3:5], atol=1e-8, rtol=0)):
        raise ValueError("Initial state disagrees with the first reference pose or wheel rates")
    def torque_at(time, state):
        index = min(round(time / d.command_sample_dt_s), len(values)-1)
        return Torques(*values[index, 1:3])
    replay = integrate(parameters, times[-1], start, torque_at)
    if len(replay) != len(values):
        raise ValueError("Replay and recording grids differ")
    predicted = np.array([[row[key] for key in ("x_m", "y_m", "yaw_rad")] for row in replay])
    odometry = wheel_odometry(times, values[:, 3], values[:, 4], parameters, values[0, 5:8])
    reference = values[:, 5:8]
    from dataclasses import asdict
    report = {"run_id": metadata["run_id"], "metadata": metadata, "parameters": asdict(parameters),
              "data_kind": metadata["data_kind"], "real_robot_validated": False,
              "scope": "One-run torque replay comparison; no calibration, tuning, hypothesis acceptance or publication",
              "m0_encoder_vs_reference": pose_errors(odometry, reference),
              "m2_torque_replay_vs_reference": pose_errors(predicted, reference),
              "csv_sha256": hashlib.sha256(Path(csv_path).read_bytes()).hexdigest(),
              "metadata_sha256": hashlib.sha256(Path(metadata_path).read_bytes()).hexdigest(),
              "v1_preservation": verify_v1()}
    for row, ideal, truth in zip(replay, odometry, reference):
        row.update(m0_encoder_x_m=float(ideal[0]), m0_encoder_y_m=float(ideal[1]), m0_encoder_yaw_rad=float(ideal[2]),
                   reference_x_m=float(truth[0]), reference_y_m=float(truth[1]), reference_yaw_rad=float(truth[2]))
    return report, replay


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("metadata", type=Path)
    parser.add_argument("--name", required=True, help="New output folder name; existing runs are never overwritten")
    args = parser.parse_args()
    verify_v1()
    report, replay = evaluate(args.recording, args.metadata)
    output = new_output_directory(args.name)
    write_json(output / "evaluation.json", report)
    write_csv(output / "replay.csv", replay)
    print(output)


if __name__ == "__main__":
    main()
