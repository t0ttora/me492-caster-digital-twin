"""Compare Python and native Simscape logs and verify both axle-torque inputs."""

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "caster-physics-python-v2"))
from twin import pose_errors


TIME_EPSILON_S = 1e-9
TORQUE_TOLERANCE_NM = 1e-9
AXLE_COLUMNS = ("tau_left_Nm", "tau_right_Nm")


def digest(path):
    sha = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            sha.update(chunk)
    return sha.hexdigest()


def read_columns(path, names):
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
    values = np.column_stack([data[key] for key in names])
    if len(values) < 2 or not np.isfinite(values).all():
        raise ValueError(f"Expected at least two finite data rows: {path}")
    if np.any(np.diff(values[:, 0]) <= 0.0):
        raise ValueError(f"Nonmonotonic timestamp sequence: {path}")
    return values


def commands_on_grid(commands, time):
    index = np.searchsorted(commands[:, 0], time + TIME_EPSILON_S, side="right") - 1
    if (index < 0).any() or time[-1] > commands[-1, 0] + TIME_EPSILON_S:
        raise ValueError("Native timestamps exceed the provided command grid")
    return commands[index, 1:]


def compare(python_dir, native_csv, output):
    python_pose_path = python_dir / "results.csv"
    torque_path = python_dir / "applied_torque.csv"
    python_manifest = json.loads((python_dir / "manifest.json").read_text())
    input_digest = digest(torque_path)
    if input_digest != python_manifest["files"]["applied_torque.csv"]:
        raise ValueError("The Python torque file disagrees with its run manifest")
    source_policy = python_manifest["solver"]["applied_torque_interpolation"]
    if source_policy.casefold() != "zoh":
        raise ValueError("Python manifest does not declare ZOH axle torques")

    command = read_columns(torque_path, ("time_s", *AXLE_COLUMNS))
    python = read_columns(python_pose_path,
                          ("time_s", "x_m", "y_m", "yaw_rad", "caster_angle_body_rad",
                           "left_wheel_rate_radps", "right_wheel_rate_radps"))
    native = read_columns(native_csv,
                          ("time_s", "axle_x_m_1", "axle_y_m_1", "axle_angles_deg_1",
                           "caster_angle_1", "left_speed_1", "right_speed_1",
                           "tau_left_Nm_1", "tau_right_Nm_1"))
    native[:, 3:5] = np.deg2rad(native[:, 3:5])
    start = max(command[0, 0], native[0, 0], python[0, 0])
    end = min(command[-1, 0], native[-1, 0], python[-1, 0])
    npg = native[(native[:, 0] >= start - TIME_EPSILON_S) & (native[:, 0] <= end + TIME_EPSILON_S)]
    ppg = python[(python[:, 0] >= start - TIME_EPSILON_S) & (python[:, 0] <= end + TIME_EPSILON_S)]
    if len(npg) < 2 or len(ppg) < 2:
        raise ValueError("No usable shared time window")

    reference_torque = commands_on_grid(command, npg[:, 0])
    delta = npg[:, 7:] - reference_torque
    torque_error = np.max(np.abs(delta), axis=0)
    native_signals = np.column_stack([
        np.interp(ppg[:, 0], native[:, 0], signal)
        for signal in (native[:, 1], native[:, 2], np.unwrap(native[:, 3]),
                       np.unwrap(native[:, 4]), native[:, 5], native[:, 6])
    ])
    residual = native_signals - ppg[:, 1:]
    residual[:, 2:4] = np.arctan2(np.sin(residual[:, 2:4]), np.cos(residual[:, 2:4]))

    coverage = bool(abs(start - command[0, 0]) <= TIME_EPSILON_S
                    and abs(end - command[-1, 0]) <= TIME_EPSILON_S)
    report = {
        "python_run_id": python_manifest["run_id"],
        "command_sha256": input_digest,
        "native_raw_sha256": digest(native_csv),
        "window_s": [float(start), float(end)],
        "native_samples": len(npg),
        "comparison_samples": len(ppg),
        "full_command_coverage": coverage,
        "applied_torque_atol_Nm": TORQUE_TOLERANCE_NM,
        "applied_torque_max_abs_error_Nm": dict(zip(("left", "right"), torque_error.tolist())),
        "equal_applied_torque": bool(coverage and np.all(torque_error <= TORQUE_TOLERANCE_NM)),
        "initial_delta": dict(zip(("x_m", "y_m", "yaw_rad", "caster_rad",
                                   "left_wheel_radps", "right_wheel_radps"), residual[0].tolist())),
        "pose_error": pose_errors(native_signals[:, :3], ppg[:, 1:4]),
        "caster_angle_rmse_rad": float(np.sqrt(np.mean(residual[:, 3]**2))),
        "wheel_speed_rmse_radps": dict(zip(("left", "right"), np.sqrt(np.mean(residual[:, 4:]**2, axis=0)).tolist())),
        "scope": "Axle-midpoint response of two synthetic model configurations at time-aligned inputs.",
    }

    output.mkdir(parents=True, exist_ok=False)
    fields = ("x_m", "y_m", "yaw_rad", "caster_rad", "left_radps", "right_radps")
    columns = ",".join(["time_s", *(f"python_{x}" for x in fields), *(f"native_{x}" for x in fields)])
    np.savetxt(output / "pose-overlay.csv", np.column_stack([ppg, native_signals]),
               delimiter=",", header=columns, comments="")
    np.savetxt(output / "applied-torque-audit.csv",
               np.column_stack([npg[:, 0], reference_torque, npg[:, 7:], delta]),
               delimiter=",", comments="",
               header="time_s,expected_left_Nm,expected_right_Nm,native_left_Nm,native_right_Nm,delta_left_Nm,delta_right_Nm")
    (output / "comparison.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("python_run", type=Path)
    parser.add_argument("native_csv", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    compare(args.python_run, args.native_csv, args.output)
