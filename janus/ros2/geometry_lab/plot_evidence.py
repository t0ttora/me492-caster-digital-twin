"""Plot case evidence from the native traces, using test-relative simulation time."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

from metrics import arrays, load_trace, stamps

TRACE = "#133746"
REQUEST = "#c36729"
BOUND = "#635b91"
SETUP = "#8592a3"
WHEELS = ("left_wheel", "right_wheel", "caster_wheel")


def setup(axis, ylabel, xlabel="Simulation time (s)"):
    axis.set_xlabel(xlabel)
    axis.set_ylabel(ylabel)
    axis.grid(alpha=.20)
    axis.spines[["right", "top"]].set_visible(False)


def lines(axis, events, labels):
    for key, label in labels:
        if key in events:
            axis.axvline(events[key], color=BOUND, alpha=.40, lw=.85, label=label)


def legend(axis):
    handles, labels = axis.get_legend_handles_labels()
    if handles:
        axis.legend(handles, labels, loc="best", fontsize=8, frameon=False)


def samples(path):
    return np.genfromtxt(path, delimiter=",", names=True, dtype=float)


def plot(run, target):
    metrics = json.loads((run/"metrics.json").read_text())
    fixture = json.loads((run/"fixture.json").read_text())
    case = fixture["case"]
    origin = metrics["termination"]["origin_sim_time_s"]
    channels = load_trace(run/"trace.jsonl")
    gt = [row for row in channels["ground_truth"] if row["time_s"] >= origin]
    request = channels["request"]
    gt_t = stamps(gt)-origin
    gt_xy = arrays(gt, ("x_m", "y_m"))
    gt_speed = np.linalg.norm(arrays(gt, ("v_mps", "lateral_mps")), axis=1)
    req_t = stamps(request, "read_time_s")-origin
    deriv = samples(run/"derived.csv")
    data_t = deriv["time_s"]
    events = metrics["events_s"]

    figure, axes = plt.subplots(3, 1, figsize=(8, 8), layout="constrained")
    first, second, third = axes
    figure.suptitle(run.name, x=.09, ha="left", weight="bold", fontsize=14)
    if case in {"M07", "O02"}:
        first.plot(*gt_xy.T, c=TRACE, label="Gazebo ground truth")
        ref = np.asarray([r["reference"] for r in request if r["reference"] is not None])
        first.plot(ref[:, 0], ref[:, 1], c=REQUEST, lw=.9, ls="--", label="Command reference")
        xmin, xmax, ymin, ymax = fixture["bounds_m"]
        first.add_patch(Rectangle((xmin, ymin), xmax-xmin, ymax-ymin,
                                  fill=False, edgecolor=SETUP, linewidth=1.4))
        first.set(xlim=(xmin-.02, xmax+.02), ylim=(ymin-.02, ymax+.02), aspect="equal")
        setup(first, "World y (m)", "World x (m)")
        second.plot(data_t, 1000*deriv["model_wall_clearance_m"], c=TRACE, label="SDF collision solids")
        second.plot(data_t, 1000*deriv["upstream_rectangle_wall_clearance_m"], c=BOUND,
                    ls="--", label="Upstream footprint rectangle")
        second.axhline(0., c=SETUP, lw=.7)
        setup(second, "Wall clearance (mm)")
        if case == "M07":
            third.plot(data_t, 1000*deriv["cross_track_m"], c=TRACE, label="Nearest reference point")
            setup(third, "Cross-track error (mm)")
        else:
            joints = channels["joint_states"]
            angles = arrays(joints, ("caster_swivel_angle_rad",))[:, 0]
            t = stamps(joints)-origin
            mask = t >= 0.
            third.plot(t[mask], np.rad2deg(np.unwrap(angles[mask])), c=TRACE, label="Passive caster swivel")
            setup(third, "Body-relative angle (°)")
    elif case == "M11":
        first.plot(gt_t, gt_xy[:, 0], c=TRACE, label="Axle x")
        first.axhline(.30, c=SETUP, ls="--", label="Surface seam")
        setup(first, "World x (m)")
        second.plot(gt_t, gt_speed, c=TRACE, label="Gazebo planar speed")
        second.step(req_t, arrays(request, ("v_mps",))[:, 0], where="post", c=REQUEST,
                    alpha=.75, label="Command request")
        lines(second, events, [("brake_step", "Brake request")])
        setup(second, "Speed (m/s)")
        third.plot(data_t, deriv["drive_left_slip_mps"], c=TRACE, label="Left wheel")
        third.plot(data_t, deriv["drive_right_slip_mps"], c=REQUEST, ls="--", label="Right wheel")
        setup(third, "Drive longitudinal slip (m/s)")
    elif case == "M12":
        first.plot(gt_t, gt_xy[:, 0], c=TRACE, label="Gazebo axle x")
        for x, label in ((.30, "Ramp entry"), (1., "Ramp exit"), (1.52, "Threshold front"),
                         (1.76, "Threshold back")):
            first.axhline(x, c=SETUP, lw=.85, alpha=.6, label=label)
        setup(first, "World x (m)")
        for feature, color in zip(("approach", "finite_ramp", "landing", "finite_threshold"),
                                  (SETUP, TRACE, BOUND, REQUEST)):
            second.plot(data_t, 1000*deriv[f"chassis_vertical_clearance_{feature}_m"],
                        color=color, label=feature.replace("_", " "))
        setup(second, "Chassis vertical gap (mm)")
        support = samples(run/"support.csv")
        for wheel, color in zip(WHEELS, (TRACE, BOUND, REQUEST)):
            third.plot(support["time_s"], support[f"{wheel}_vertical_N"], alpha=.8, lw=.75,
                       color=color, label=wheel.replace("_", " "))
        setup(third, "Solver vertical load (N)")
    else:
        first.plot(gt_t, gt_xy[:, 0], c=TRACE, label="Gazebo axle")
        fixture_native = run/"fixture_native_pose.csv"
        if fixture_native.exists():
            values = samples(fixture_native)
            native_t = values["sim_time_ns"]/1e9-origin
            mask = (native_t >= 0.) & (native_t <= gt_t[-1])
            first.plot(native_t[mask], values["y_m"][mask], c=BOUND, label="Crossing fixture y")
        setup(first, "World position (m)")
        second.plot(gt_t, gt_speed, c=TRACE, label="Gazebo planar speed")
        second.step(req_t, arrays(request, ("v_mps",))[:, 0], where="post", c=REQUEST,
                    alpha=.75, label="Command request")
        lines(second, events, [("pedestrian_detected", "Hazard received"),
                              ("obstacle_detected", "Hazard received"),
                              ("resume_gate_open", "Clear + consent")])
        setup(second, "Speed (m/s)")
        field = "fixture_conservative_aabb_separation_m"
        if field in deriv.dtype.names:
            third.plot(data_t, 1000*deriv[field], c=TRACE, label="Projection gap, synchronized poses")
        setup(third, "Obstacle separation bound (mm)")

    for axis in axes:
        legend(axis)
    figure.savefig(target, dpi=160, metadata={"Title": run.name})
    plt.close(figure)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("plot", type=Path)
    args = parser.parse_args()
    plot(args.run, args.plot)
