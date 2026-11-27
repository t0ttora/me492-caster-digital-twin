#!/usr/bin/env python3
"""Draw the two independently observed Q04 trajectories and terminal gates."""

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def world_and_events(folder):
    world = []
    events = {}
    with (folder / "telemetry.jsonl").open() as stream:
        for line in stream:
            row = json.loads(line)
            if row["source"] == "truth":
                world.append(row)
            elif row["source"] == "event":
                events[row["event"]] = row["read_time_s"]
    return world, events


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("run", type=Path)
    cli.add_argument("original", type=Path)
    cli.add_argument("image", type=Path)
    args = cli.parse_args()
    old_world, _ = world_and_events(args.original)
    new_world, events = world_and_events(args.run)
    result = json.loads((args.run / "acceptance.json").read_text())["measurements"]
    fixture = json.loads((args.original / "fixture.json").read_text())
    goal_x, goal_y, _ = fixture["dock_nominal_axle_xy_yaw"]
    stage_completed = min(new_world, key=lambda pose: abs(pose["time_s"] - events["optical_center"]))

    ink, muted, rust, teal = "#183137", "#4a6262", "#ad583d", "#0c7775"
    plt.rcParams.update({"font.family": "DejaVu Sans", "text.color": ink, "axes.labelcolor": ink,
                         "axes.titlecolor": ink, "xtick.color": muted, "ytick.color": muted})
    fig = plt.figure(figsize=(13.4, 6.2), facecolor="#f8f8f2")
    fig.subplots_adjust(left=.08, right=.95, top=.77, bottom=.20, wspace=.36, hspace=.79)
    panels = fig.add_gridspec(2, 2, width_ratios=(1.6, 1))
    xy = fig.add_subplot(panels[:, 0])
    xy.set_title("Bağımsız simülatör poz izi", loc="left", fontsize=12, fontweight="bold")
    for rows, label, color in ((old_world, "İlk optik yaklaşım", rust), (new_world, "QR ile aşamalı yaklaşım", teal)):
        xy.plot([p["x_m"] for p in rows], [p["y_m"] * 1000 for p in rows],
                color=color, label=label, linewidth=2.2)
        xy.scatter(rows[-1]["x_m"], rows[-1]["y_m"] * 1000, c=color, s=42, zorder=4)
    xy.scatter(stage_completed["x_m"], stage_completed["y_m"] * 1000, c=teal, marker="s", s=34, zorder=4)
    xy.annotate("Ön hizalama bitti", (stage_completed["x_m"], stage_completed["y_m"]*1000),
                xytext=(.57, -39), arrowprops={"arrowstyle": "-", "color": muted}, color=muted, fontsize=9)
    xy.scatter(goal_x, goal_y * 1000, c=ink, marker="+", s=100, zorder=5, label="Nominal son poz")
    xy.set(xlim=(-.02, 1.05), ylim=(-50, 155), xlabel="Dünya X (m)", ylabel="Dünya Y (mm)")
    xy.grid(alpha=.21)
    xy.legend(loc="upper right", fontsize=9, frameon=False)

    comparators = (
        ("Son mekanik konum hatası (mm)",
         [result["original_Q04_terminal_position_error_m"]*1000, result["terminal_position_error_m"]*1000],
         20, 52, ".2f"),
        ("Mutlak son yaw hatası (rad)",
         [abs(result["original_Q04_terminal_yaw_error_rad"]), abs(result["terminal_yaw_error_rad"])],
         .05, .105, ".5f"),
    )
    for index, (title, values, limit, width, formatter) in enumerate(comparators):
        axis = fig.add_subplot(panels[index, 1])
        axis.set_title(title, loc="left", fontsize=11, fontweight="bold")
        axis.barh([1, 0], values, color=[rust, teal], height=.50)
        axis.axvline(limit, color=ink, ls="--", lw=1.2)
        axis.set(xlim=(0, width), yticks=[1, 0], yticklabels=["İlk", "Aşamalı"])
        axis.text(limit + width*.018, .35, "Kabul sınırı", color=muted, fontsize=7, rotation=0)
        for y, value in zip([1, 0], values):
            axis.text(value + width*.025, y, f"{value:{formatter}}".replace(".", ","), va="center", fontsize=9)
        axis.grid(axis="x", alpha=.19)
        for side in ("top", "right", "left"):
            axis.spines[side].set_visible(False)

    fig.text(.08, .924, "Q04 · Optik duruş ile mekanik hizalanma", fontsize=21, fontweight="bold")
    fig.text(.08, .868, "Eş başlangıç pozu ve QR geometrisi · Ubuntu / ROS 2 Jazzy / Gazebo Harmonic",
             fontsize=11, color=muted)
    depth = f"{result['terminal_optical_depth_m'] * 1000:.2f}".replace(".", ",")
    mid = f"{result['terminal_normalized_center_error'] * 100:+.3f}".replace(".", ",")
    dwell = round(result["terminal_world_dwell_s"] * 1000)
    fig.text(.08, .078, f"Optik derinlik {depth} mm · Merkez hatası {mid}% · Mekanik kabul bandında {dwell} ms",
             fontsize=10, color=ink, fontweight="bold")
    fig.text(.08, .041, "İzler Gazebo dünya pozundan hesaplandı. Kamera girdisi canlı QR. Fiziksel dock ve BMS pilot kapsamındadır.",
             fontsize=9, color=muted)
    fig.savefig(args.image, dpi=155, facecolor=fig.get_facecolor())
    plt.close(fig)


if __name__ == "__main__":
    main()
