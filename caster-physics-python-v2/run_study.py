"""Run reproducible synthetic comparisons without changing V1 or past outputs."""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import sys

import numpy as np

from twin import ROOT, MANEUVERS, PROFILES, SURFACES, DESIGNS, run_case, verify_v1, controller_gains, load_parameters


def new_output_directory(name):
    if not name or Path(name).name != name or name in {".", ".."}:
        raise ValueError("Output name must be one directory name")
    parent = ROOT / "outputs"
    if parent.is_symlink():
        raise ValueError("Output root must not be a symlink")
    parent.mkdir(exist_ok=True)
    output = parent / name
    output.mkdir(exist_ok=False)
    return output


def write_json(path, data):
    with path.open("x", encoding="utf-8") as file:
        json.dump(data, file, ensure_ascii=False, indent=2, allow_nan=False)
        file.write("\n")


def write_csv(path, rows):
    with path.open("x", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)


def plot_case(rows, path, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(1, 3, figsize=(13, 3.7))
    def col(key):
        return [row[key] for row in rows]
    axes[0].plot(col("ref_x_m"), col("ref_y_m"), "--", label="Command path / ideal M0")
    axes[0].plot(col("x_m"), col("y_m"), label="M2 synthetic")
    axes[0].plot(col("m0_encoder_x_m"), col("m0_encoder_y_m"), ":", label="M0 wheel odometry")
    axes[0].set(xlabel="Axle x [m]", ylabel="Axle y [m]", aspect="equal")
    axes[0].legend(fontsize=7)
    axes[1].plot(col("time_s"), col("ref_yaw_rad"), "--", label="Reference")
    axes[1].plot(col("time_s"), col("yaw_rad"), label="M2 yaw")
    axes[1].plot(col("time_s"), col("caster_angle_body_rad"), label="Relative caster")
    axes[1].set(xlabel="Time [s]", ylabel="Angle [rad]")
    axes[1].legend(fontsize=7)
    axes[2].plot(col("time_s"), col("command_speed_mps"), "--", label="Reference speed")
    axes[2].plot(col("time_s"), col("body_long_velocity_mps"), label="COM forward speed")
    axes[2].set(xlabel="Time [s]", ylabel="Speed [m/s]")
    axes[2].legend(fontsize=7)
    figure.suptitle(title, fontsize=10)
    figure.tight_layout()
    figure.savefig(path, dpi=150)
    plt.close(figure)


def main():
    args = sys.argv[1:]
    janus_flags = {"--case", "--variant", "--output-dir", "--publish-commands", "--jobs"}
    janus_groups = {"p0", "additional"}
    group = next((arg.partition("=")[2] if "=" in arg else args[i+1]
                  for i, arg in enumerate(args)
                  if arg.startswith("--group=") or (arg == "--group" and i+1 < len(args))), None)
    if any(arg.split("=")[0] in janus_flags for arg in args) or group in janus_groups:
        from run_janus import main as run_janus
        run_janus(args)
        return
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--group", choices=("core", "design", "all"), default="core")
    parser.add_argument("--name", default=datetime.now(timezone.utc).strftime("study-%Y%m%dT%H%M%S%fZ"))
    args = parser.parse_args()
    preservation = verify_v1()
    output = new_output_directory(args.name)
    cases = []
    if args.group in {"core", "all"}:
        cases += [(maneuver, profile, surface, "nominal")
                  for surface in SURFACES for maneuver in MANEUVERS for profile in PROFILES]
    if args.group in {"design", "all"}:
        cases += [("arc_turn", profile, surface, design)
                  for surface in SURFACES for design in DESIGNS for profile in PROFILES
                  if design != "nominal" or args.group == "design"]
    results = []
    for index, (maneuver, profile, surface, design) in enumerate(cases, 1):
        run_id = f"{surface}__{maneuver}__{profile}__{design}"
        print(f"[{index}/{len(cases)}] {run_id}", flush=True)
        rows, metrics, parameters = run_case(maneuver, profile, surface, design)
        csv_path = output / f"{run_id}.csv"
        write_csv(csv_path, rows)
        plot_case(rows, output / f"{run_id}.png", run_id)
        results.append({"run_id": run_id, "surface": surface, "maneuver": maneuver,
                        "profile": profile, "design": design, "parameters": parameters,
                        "metrics": metrics, "csv_sha256": hashlib.sha256(csv_path.read_bytes()).hexdigest()})
    checks = {
        "v1_preserved": verify_v1()["passed"],
        "energy_balance_below_1e-5J": all(run["metrics"]["energy_balance_max_J"] < 1e-5 for run in results),
        "passive_contact": all(run["metrics"]["dissipation_min_W"] >= -1e-10 for run in results),
        "torque_limit_0.5Nm": all(run["metrics"]["torque_peak_Nm"] <= .5 for run in results),
        "all_runs_completed": all(run["metrics"]["completion_time_s"] is not None for run in results),
    }
    manifest = {
        "schema_version": "arche-python-v2.3", "data_kind": "synthetic", "real_robot_validated": False,
        "created_utc": datetime.now(timezone.utc).isoformat(), "python_version": platform.python_version(),
        "numpy_version": np.__version__, "v1_preservation": preservation,
        "controller": "synthetic Lyapunov tracking, terminal position/yaw modes, wheel PI with conditional anti-windup; +-0.5Nm, 100Hz ZOH",
        "controller_constants": {**controller_gains(load_parameters()), "terminal_enter_align_m": .006,
                                 "terminal_reacquire_m": .012, "speed_limit_mps": .25,
                                 "yaw_limit_radps": 1., "terminal_observation_s": 8.},
        "gain_design_targets": {"longitudinal_time_constant_s": .5, "lateral_nominal_speed_mps": .15,
                                "lateral_poles_per_s": [-1., -1.], "sampled_wheel_poles": [.8, .5],
                                "terminal_rho_time_constant_s": .5, "terminal_angle_time_constant_s": 1/3,
                                "status": "selected design requirements, not measurements or historic derivation claims"},
        "solver": "V1 fixed-step RK4, 0.0005s", "initial_caster_angle_rad": np.pi/4,
        "source_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in ROOT.glob("*.py")},
        "parameter_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'parameters').glob('*.json')},
        "limitations": ["No measured surfaces or calibrated robot", "M0 uses the SAME wheel rates for odometry",
                        "M2 as synthetic truth does not establish RQ1 superiority", "No physical caster winner",
                        "Radius sweeps retain wheel mass and scale caster roll inertia with radius squared; other inertias remain provisional",
                        "Smooth profile takes 1.875x scheduled time at the same peak reference speed",
                        "One deterministic run per cell, not independent physical repeats",
                        "Eight-second terminal recovery horizon; original one-second completion also reported",
                        "Continuous ideal-kinematic proofs do not establish saturated sampled M2 closed-loop stability"],
        "checks": checks, "all_checks_passed": all(checks.values()), "runs": results,
    }
    write_json(output / "study.json", manifest)
    lines = ["# V2 synthetic study", "", "Synthetic development evidence only. No robot calibration or physical validation.", "",
             "| Surface | Maneuver | Design | Profile | Position RMSE m | Path RMSE m | Yaw RMSE rad | Completion s | Jerk RMS m/s3 |",
             "| --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: |"]
    for run in results:
        m = run["metrics"]
        completion = "not reached" if m["completion_time_s"] is None else f"{m['completion_time_s']:.2f}"
        lines.append(f"| {run['surface']} | {run['maneuver']} | {run['design']} | {run['profile']} | "
                     f"{m['tracking']['position_rmse_m']:.5f} | {m['path_rmse_m']:.5f} | "
                     f"{m['tracking']['yaw_rmse_rad']:.5f} | {completion} | {m['jerk_rms_mps3']:.3f} |")
    lines += ["", "## Interpretation", "", "Compare profiles within each surface/maneuver/design, including duration and failures.",
              "Spatial error uses the nearest of 501 points on the common path. Synchronized tracking error uses each profile's own clock.",
              "Jerk is a finite-difference estimate at 100Hz; it depends on sampling and includes the stop interval.",
              "Motor work is signed mechanical axle work, not battery consumption.",
              "", "All numerical checks passed: " + str(all(checks.values()))]
    (output / "summary.md").write_text("\n".join(lines) + "\n")
    print(output, flush=True)
    if not all(checks.values()):
        raise SystemExit("Numerical checks failed; inspect study.json without relaxing thresholds")


if __name__ == "__main__":
    main()
