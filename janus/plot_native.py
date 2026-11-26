"""Replot Elo native workspace exports and calculate measured stop/gate data."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from matplotlib import pyplot as plt


def decorate(axis, title, xlabel="Time (s)"):
    axis.set(title=title, xlabel=xlabel)
    axis.grid(alpha=0.2)


def first_dwell(time, valid, begin, duration):
    began = None
    for i in range(begin, len(time)):
        if not valid[i]:
            began = None
        elif began is None:
            began = i
        elif time[i] - time[began] >= duration - 1e-9:
            return began, i
    return None


def examine(run, output):
    raw = run / "signals_raw.csv"
    report = json.loads((run / "report.json").read_text())
    data = np.genfromtxt(raw, delimiter=",", names=True)
    t = data["time_s"]
    if (not report["completed"] or not np.allclose(np.diff(t), report["sensor_log_s"], atol=1e-8, rtol=0)
            or not np.isclose(t[0], 0) or not np.isclose(t[-1], report["end_s"])):
        raise ValueError("Expected one complete native sampled run")
    x, y = data["axle_x_m_1"], data["axle_y_m_1"]
    yaw = np.unwrap(np.deg2rad(data["axle_angles_deg_1"]))
    caster, swivel = data["caster_angle_1"], data["swivel_speed_1"]
    speed, yaw_rate = data["control_30"], data["control_31"]
    v_cmd, w_cmd = data["control_3"], data["control_4"]
    phase = data["control_5"]
    gate = data["control_16"]
    is_qr = report["mode"] in {3, 4, 5, 6}
    valid = (data["control_12"] > .5) & (data["control_13"] > .5)

    plot, axes = plt.subplots(4, 2, figsize=(12, 10), layout="constrained")
    plot.suptitle(f"Native Simscape workspace export · {report['scenario']}\n"
                  f"R2026b · ode4 {report['step_s'] * 1e6:g} µs · {report['sensor_log_s'] * 1e3:g} ms logged")
    axis = axes[0, 0]
    axis.plot(x, y, color="#126e82")
    axis.scatter(x[[0, -1]], y[[0, -1]], color=["#189663", "#a2482d"], s=24)
    decorate(axis, "Measured axle path (m)", "World x (m)")
    axis.set_ylabel("World y (m)")
    axis.set_aspect("equal", adjustable="datalim")
    axis = axes[0, 1]
    axis.plot(t, np.rad2deg(yaw), label="Yaw")
    axis.plot(t, caster, label="Relative caster angle")
    decorate(axis, "Yaw and caster angle (deg)")
    axis.legend(loc="best")
    axis = axes[1, 0]
    axis.plot(t, data["left_speed_1"], label="Left")
    axis.plot(t, data["right_speed_1"], label="Right")
    axis.plot(t, swivel, label="Caster swivel")
    decorate(axis, "Wheel and swivel rates (rad/s)")
    axis.legend(loc="best")
    axis = axes[1, 1]
    axis.plot(t, data["tau_left_Nm_1"], label="Left")
    axis.plot(t, data["tau_right_Nm_1"], label="Right")
    decorate(axis, "Applied axle torque (N m)")
    axis.legend(loc="best")
    for wheel, name in (("L", "Left"), ("R", "Right"), ("C", "Caster")):
        axes[2, 0].plot(t, data[f"normal_{wheel}_3"], label=name)
        axes[2, 1].plot(t, 1000 * data[f"pen_{wheel}_1"], label=name)
    decorate(axes[2, 0], "Per-wheel normal force (N)")
    decorate(axes[2, 1], "Contact penetration (mm)")
    axes[2, 0].legend(loc="best")
    axes[2, 1].legend(loc="best")
    axis = axes[3, 0]
    axis.plot(t, v_cmd, label="Linear request (m/s)")
    axis.plot(t, w_cmd, label="Yaw request (rad/s)")
    decorate(axis, "Controller requests")
    axis.legend(loc="best")
    axis = axes[3, 1]

    metrics = {
        "scenario": report["scenario"],
        "samples": len(t),
        "solver_step_s": report["step_s"],
        "sensor_sample_s": report["sensor_log_s"],
        "native_raw_sha256": hashlib.sha256(raw.read_bytes()).hexdigest(),
        "latched_gate_count": int(np.max(gate)),
        "final_axle_position_m": [float(x[-1]), float(y[-1])],
        "final_yaw_rad": float(yaw[-1]),
        "final_measured_speed_mps": float(speed[-1]),
        "final_measured_yaw_rate_radps": float(yaw_rate[-1]),
        "case_limit": "Native model evidence. QR frames come from synthetic projection.",
    }
    if is_qr:
        qr = data["control_10"]
        axis.axhspan(-.04, .04, color="#dbebec", label="Outer band")
        axis.axhspan(-.02, .02, color="#b2d8dc", label="Inner band")
        axis.plot(t, np.where(valid, qr, np.nan), label="Valid normalized error", color="#126e82")
        stage_axis = axis.twinx()
        stage_axis.step(t, phase, color="#80512d", alpha=0.56, where="post")
        stage_axis.set_ylabel("Controller phase code")
        decorate(axis, "Projected QR error and phase")
        axis.legend(loc="best")
        advance = np.flatnonzero(v_cmd > 1e-8)
        seen = np.flatnonzero(valid)
        drop = np.flatnonzero(valid[:-1] & ~valid[1:]) + 1
        if len(seen):
            metrics.update(first_valid_s=float(t[seen[0]]), first_error=float(qr[seen[0]]),
                           first_yaw_request_radps=float(w_cmd[seen[0]]))
        if len(advance):
            metrics.update(first_advance_s=float(t[advance[0]]),
                           max_abs_qr_error_in_advance=float(np.max(np.abs(qr[advance]))))
        metrics["forward_samples_with_invalid_qr"] = int(np.count_nonzero((v_cmd > 1e-8) & ~valid))
        if len(drop):
            first_drop = drop[0]
            off_samples = np.flatnonzero(v_cmd[first_drop:] <= 1e-8)
            first_off = first_drop + off_samples[0] if len(off_samples) else None
            stop = None if first_off is None else first_dwell(
                t, (np.abs(speed) <= .02) & (np.abs(yaw_rate) <= .05), first_off, .25
            )
            metrics["first_loss"] = {"first_invalid_s": float(t[first_drop]),
                                     "forward_zero_s": None if first_off is None else float(t[first_off]),
                                     "last_valid_forward_mps": float(v_cmd[first_drop - 1])}
            if stop:
                begin, verified = stop
                travel = np.hypot(np.diff(x[first_drop:begin + 1]), np.diff(y[first_drop:begin + 1]))
                metrics["first_loss"].update(stop_speed_onset_s=float(t[begin]),
                                             speed_dwell_verified_s=float(t[verified]),
                                             traveled_to_stop_m=float(np.sum(travel)))
    else:
        axis.step(t, gate, label="Gates accepted", where="post")
        decorate(axis, "Route gate progression")
        axis.legend(loc="best")
        changes = np.flatnonzero(gate[1:] > gate[:-1]) + 1
        metrics["gate_times_s"] = [float(t[index]) for index in changes]
    output.mkdir(parents=True, exist_ok=True)
    plot.savefig(output / "overview.png", dpi=160)
    plt.close(plot)
    (output / "native-metrics.json").write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("native_run", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    examine(args.native_run, args.output)
