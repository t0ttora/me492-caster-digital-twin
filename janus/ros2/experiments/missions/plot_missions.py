"""Plot recorded mission evidence from the same raw logs consumed by the verifier."""
import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from contract import CASTER_ERROR_LIMIT_RAD, DRIVE_RADIUS_M, GUIDES, dock_clearance, wrap
from verify_mission import Trace

COLORS = {"truth": "#203D68", "odom": "#279982", "request": "#C16C23", "guard": "#7D52A0",
          "event": "#8C929B", "vline": "#525C67"}
EVENTS = {
    "Q15": ("aligned", "zero_motion", "contact", "charge_timeout"),
    "Q16": ("aligned", "zero_motion", "contact", "charge_confirmed"),
    "Q18": ("undock_authorized", "charge_cut_confirmed", "reverse_enabled", "contact_released", "exit_stopped"),
    "O07": ("transit_started", "transit_preempted", "transit_stopped", "dock_started", "charge_confirmed"),
}


def load(path):
    with (path / "telemetry.jsonl").open() as source:
        return Trace(json.loads(line) for line in source)


def xy(trace, source, field, origin):
    rows = [r for r in trace.timed[source] if r["time_s"] >= origin]
    return [r["time_s"]-origin for r in rows], [r[field] for r in rows]


def style_axis(axis):
    axis.grid(alpha=.22, color="#A2A6AD", linewidth=.65)
    axis.spines[["top", "right"]].set_visible(False)
    axis.tick_params(labelsize=8)


def markup(axis, trace, origin, markers):
    for event, label in markers:
        stamps = trace.events[event]
        for row in stamps:
            stamp = row["time_s"]-origin
            if stamp < 0.:
                continue
            axis.axvline(stamp, color=COLORS["event"], linewidth=.8)
            axis.text(stamp, .96, label.format(**row), rotation=90, transform=axis.get_xaxis_transform(),
                      va="top", ha="right", fontsize=7, color=COLORS["vline"])


def charge_plot(paths, output):
    figures, axes = plt.subplots(2, 2, figsize=(12.4, 7.5), layout="constrained")
    for index, case in enumerate(("Q15", "Q16")):
        if case not in paths:
            continue
        trace = load(paths[case])
        if trace.start is None:
            continue
        ax = axes[index][0]
        for source, color, label in (("truth", "truth", "Gazebo world"), ("odom", "odom", "Wheel odom"),
                                     ("guard_output", "guard", "Guard output")):
            ax.plot(*xy(trace, source, "v_mps", trace.start), label=label, color=COLORS[color], linewidth=1.0)
        ax.set(ylabel="Forward speed (m/s)", xlabel="Simulation time from case start (s)")
        ax.set_title(f"{case}   Approach", loc="left", fontweight="bold")
        style_axis(ax)
        ax.legend(loc="upper right", fontsize=8, frameon=False)
        right = axes[index][1]
        origin = trace.when("aligned")
        if origin is None:
            continue
        for source, color, style in (("truth", "truth", "-"), ("guard_output", "guard", "--")):
            right.plot(*xy(trace, source, "v_mps", origin), linewidth=1.1, linestyle=style, color=COLORS[color])
        for side, color in (("left", "#E6A062"), ("right", "#9A6B23")):
            t, values = xy(trace, "joint", side + "_wheel_radps", origin)
            right.plot(t, [DRIVE_RADIUS_M*v for v in values], linewidth=.8, color=color)
        markup(right, trace, origin, [(name, name.replace("_", " ")) for name in EVENTS[case]])
        right.set(xlabel="Simulation time from alignment (s)", ylabel="Body / drive rim speed (m/s)")
        right.set_title(f"{case}   Station hold", loc="left", fontweight="bold")
        style_axis(right)
    figures.suptitle("JANUS native docking and virtual charger evidence", fontweight="bold", fontsize=15)
    figures.savefig(output / "charge-evidence.png", dpi=165)
    plt.close(figures)


def undock_plot(path, output):
    trace = load(path)
    start = trace.when("undock_authorized")
    finish = trace.when("exit_stopped")
    if start is None:
        return
    end = finish or trace.timed["truth"][-1]["time_s"]
    rows = trace.slice("truth", start, end)
    paired = [(r, trace.nearest("joint", r["time_s"])) for r in rows]
    paired = [(r, j) for r, j in paired if j]
    guides = json.loads((path / "world.json").read_text()).get("dock_guides_xy_size_si") or GUIDES
    times = [r["time_s"]-start for r, _ in paired]
    f, axes = plt.subplots(3, 1, sharex=True, figsize=(10.5, 8), layout="constrained")
    ax, cas, gap = axes
    for source, color in (("truth", "truth"), ("odom", "odom"), ("guard_output", "guard")):
        ax.plot(*xy(trace, source, "v_mps", start), label=source.replace("_", " "), color=COLORS[color])
    ax.set_ylabel("Forward speed (m/s)")
    ax.legend(loc="lower right", fontsize=8, frameon=False, ncols=3)
    cas.plot(times, [math.degrees(abs(wrap(j["caster_swivel_angle_rad"]-math.pi))) for _, j in paired],
             color=COLORS["truth"])
    cas.axhline(math.degrees(CASTER_ERROR_LIMIT_RAD), color=COLORS["request"], linestyle="--", linewidth=.8)
    cas.set_ylabel("Caster error to reverse (°)")
    gap.plot(times, [1000*dock_clearance(r, j, guides) for r, j in paired], color=COLORS["odom"])
    gap.set(ylabel="Planar clearance (mm)", xlabel="Simulation time from laboratory undock authorization (s)")
    for axis in axes:
        style_axis(axis)
    markup(ax, trace, start, [(name, name.replace("_", " ")) for name in EVENTS["Q18"][1:]])
    f.suptitle("Q18   Native backout, caster and station clearance", fontweight="bold", fontsize=15)
    f.savefig(output / "undock-evidence.png", dpi=165)
    plt.close(f)


def route_plot(paths, output):
    selected = [case for case in ("O01", "O06", "O07") if case in paths]
    if not selected:
        return
    f, axes = plt.subplots(len(selected), 1, squeeze=False, figsize=(10.5, 2.55*len(selected)),
                           layout="constrained")
    for axis, case in zip(axes[:, 0], selected):
        trace = load(paths[case])
        if trace.start is None:
            continue
        axis.plot(*xy(trace, "truth", "x_m", trace.start), color=COLORS["truth"], label="Independent Gazebo x")
        markers = ([(name, name.replace("_", " ")) for name in EVENTS[case][1:]]
                   if case in EVENTS else [("arrived", "{mission_id}")])
        markup(axis, trace, trace.start, markers)
        axis.set(ylabel="World x (m)", xlabel="Simulation time from case start (s)")
        axis.set_title(case, loc="left", fontweight="bold")
        style_axis(axis)
    f.suptitle("JANUS mission progress from Gazebo world pose", fontweight="bold", fontsize=15)
    f.savefig(output / "route-evidence.png", dpi=165)
    plt.close(f)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("runs", nargs="+", help="CASE=RUN_DIRECTORY")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    runs = {case: Path(path) for case, path in (assignment.split("=", 1) for assignment in args.runs)}
    if {"Q15", "Q16"} & runs.keys():
        charge_plot(runs, args.output)
    if "Q18" in runs:
        undock_plot(runs["Q18"], args.output)
    route_plot(runs, args.output)
