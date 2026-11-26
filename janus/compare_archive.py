"""Compare the archived Simscape alignment replay with a fresh V2.3 torque replay.

Use the uncalibrated configuration of each plant. Relative native position
comes from the archived World-to-body measurement. This diagnostic compares
responses to the two logged axle inputs. Contact, vertical dynamics and
coordinate reference differences preclude a model-equivalence verdict.
"""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "caster-physics-python-v2"))
from twin import State, Torques, integrate, load_parameters, pose_errors, verify_v1


ARCHIVE = ROOT / "models" / "simscape-v6"
CSV = ARCHIVE / "evidence" / "full_replay.csv"
MANIFEST = ARCHIVE / "evidence" / "replay_manifest_enriched.json"
MODEL = ARCHIVE / "evidence_latest_alignment_replay.slx"


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def column(records, name):
    return np.array([record[name] for record in records])


def rms(delta):
    return float(np.sqrt(np.mean(delta**2)))


def write_plot(path, time, native, python, aligned, native_force, python_force):
    from matplotlib import pyplot as plt

    figure, axes = plt.subplots(3, 2, figsize=(11, 8), layout="constrained")
    panels = (
        (time, native[0], python[0], "World x displacement (m)"),
        (time, native[1], python[1], "World y displacement (m)"),
        (time, np.rad2deg(native[2]), np.rad2deg(python[2]), "World yaw change (deg)"),
        (time, np.rad2deg(native[3]), np.rad2deg(python[3]), "Caster angle (deg)"),
        (time, native[4], python[4], "Caster swivel rate (rad/s)"),
        (time, native_force, python_force, "Total support force (N)"),
    )
    for axis, (t, reference, replay, title) in zip(axes.flat, panels):
        axis.plot(t, reference, label="Simscape archive", color="#126e82", linewidth=1.5)
        axis.plot(t, replay, label="Python V2.3 replay", color="#a2482d", linewidth=1.3)
        axis.axvspan(aligned[0], aligned[1], color="#bac6cc", alpha=0.32)
        axis.set(title=title, xlabel="Elapsed time (s)")
        axis.grid(alpha=0.2)
    axes[0, 0].legend(loc="best")
    figure.suptitle("Caster alignment with equal axle torque · 0.01 N m per wheel\n"
                    "Shading marks the retained window of the original Simscape log")
    figure.savefig(path, dpi=160)
    plt.close(figure)


def run(output):
    manifest = json.loads(MANIFEST.read_text())
    if not manifest["completed"] or sha256(MODEL) != manifest["replay_model_sha256"]:
        raise ValueError("Archive does not match its completed model manifest")
    archive = np.genfromtxt(CSV, delimiter=",", names=True)
    native_time = archive["time_s"]
    expected_end = float(manifest["end_time"])
    if (native_time[0] != 0 or not np.isclose(native_time[-1], expected_end)
            or not np.allclose(np.diff(native_time), 1e-4, rtol=0, atol=1e-10)
            or not all(np.all(np.isfinite(archive[key])) for key in archive.dtype.names)):
        raise ValueError("Archived data is incomplete or does not match the 100 μs sample grid")

    d = load_parameters()
    torque = Torques(float(manifest["commands"]["left_wheel_torque_Nm"]),
                     float(manifest["commands"]["right_wheel_torque_Nm"]))
    yaw = np.deg2rad(archive["robot_angles_c1"])
    initial_caster = np.deg2rad(float(manifest["initial_caster_deg"]))
    start = State.from_vector([
        d.dynamics.body_com_m[0], d.dynamics.body_com_m[1], 0, initial_caster,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
    ])
    records = integrate(d, expected_end, start, lambda _time, _state: torque)
    time = column(records, "time_s")

    def native(name):
        return np.interp(time, native_time, archive[name])

    native_pose = np.column_stack([native("robot_x_c1"), native("robot_y_c1"),
                                   np.interp(time, native_time, np.unwrap(yaw))])
    native_pose -= native_pose[0]
    python_pose = np.column_stack([column(records, key) for key in ("x_m", "y_m", "yaw_rad")])
    python_pose -= python_pose[0]
    native_caster = np.deg2rad(native("caster_angle_c1"))
    python_caster = column(records, "caster_angle_body_rad")
    native_swivel = native("swivel_speed_c1")
    python_swivel = column(records, "caster_relative_rate_radps")
    # The archive stores the combined normal-force vector. Use its vertical component.
    native_force = native("normal_total_c3")
    python_force = sum(column(records, f"normal_{wheel}_N") for wheel in ("left", "right", "caster"))
    left_native, right_native = native("left_speed_c1"), native("right_speed_c1")
    left_python, right_python = (column(records, f"{wheel}_wheel_rate_radps") for wheel in ("left", "right"))
    native_roll = native("caster_roll_speed_c1")
    python_roll = column(records, "caster_roll_rate_radps")
    aligned = manifest["original_retained_interval_s"]
    tail = (time >= aligned[0]) & (time <= aligned[1])

    def metrics(window):
        return {
            "samples": int(np.count_nonzero(window)),
            **pose_errors(python_pose[window], native_pose[window]),
            "caster_angle_rmse_rad": rms(python_caster[window] - native_caster[window]),
            "swivel_rate_rmse_radps": rms(python_swivel[window] - native_swivel[window]),
            "swivel_rate_max_abs_delta_radps": float(np.max(np.abs(python_swivel[window] - native_swivel[window]))),
            "left_wheel_rate_rmse_radps": rms(left_python[window] - left_native[window]),
            "right_wheel_rate_rmse_radps": rms(right_python[window] - right_native[window]),
            "caster_roll_rate_rmse_radps": rms(python_roll[window] - native_roll[window]),
            "simscape_mean_total_support_N": float(np.mean(native_force[window])),
            "python_mean_total_support_N": float(np.mean(python_force[window])),
        }

    report = {
        "run_id": "V6-20261116-v23-archive-torque-comparison",
        "scope": "New Python replay compared to a native archive dated 2026-11-16",
        "plant_limitations": [
            "Native is 3D penalty contact with SmoothStickSlip. Python is planar with quasi-static support loads.",
            "Native displacement is from the archived World-to-body signal. Python displacement is from the drive axle.",
            "Neither parameter set is hardware-calibrated. Position difference can include measurement-frame effects.",
            "This is not a solver-step refinement test. Swivel-rate differences compare different plant formulations.",
        ],
        "input": {"left_axle_Nm": torque.left_Nm, "right_axle_Nm": torque.right_Nm,
                  "initial_caster_deg": float(manifest["initial_caster_deg"])},
        "native_archive_rows": int(len(archive)),
        "native_archive_csv_sha256": sha256(CSV),
        "native_archive_model_sha256": sha256(MODEL),
        "native_archive_executed_at": manifest["executed_at"],
        "native_archive_fixed_step_s": float(manifest["fixed_step"]),
        "comparison_grid_s": d.dynamics.result_sample_dt_s,
        "python_solver_step_s": d.dynamics.solver_step_s,
        "original_retained_interval_s": aligned,
        "full_reconstructed_window": metrics(np.ones_like(time, dtype=bool)),
        "overlap_with_original_retained_window": metrics(tail),
        "endpoint": {"simscape_x_m": float(native_pose[-1, 0]), "python_x_m": float(python_pose[-1, 0]),
                     "simscape_caster_deg": float(np.rad2deg(native_caster[-1])),
                     "python_caster_deg": float(np.rad2deg(python_caster[-1]))},
        "v1_hashes": verify_v1(),
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "metrics.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    with (output / "commands.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(("time_s", "tau_left_Nm", "tau_right_Nm"))
        writer.writerows((t, torque.left_Nm, torque.right_Nm) for t in time)
    with (output / "overlay.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        signals = {"x_m": (native_pose[:, 0], python_pose[:, 0]),
                   "y_m": (native_pose[:, 1], python_pose[:, 1]),
                   "yaw_rad": (native_pose[:, 2], python_pose[:, 2]),
                   "caster_relative_rad": (native_caster, python_caster),
                   "caster_swivel_radps": (native_swivel, python_swivel),
                   "left_wheel_radps": (left_native, left_python),
                   "right_wheel_radps": (right_native, right_python),
                   "caster_roll_radps": (native_roll, python_roll),
                   "support_total_N": (native_force, python_force)}
        writer.writerow(["time_s", *[f"{plant}_{signal}" for signal in signals for plant in ("simscape", "python")]])
        writer.writerows(zip(time, *(array for both in signals.values() for array in both)))
    write_plot(output / "overlay.png", time, (*native_pose.T, native_caster, native_swivel),
               (*python_pose.T, python_caster, python_swivel), aligned, native_force, python_force)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    run(parser.parse_args().output)
