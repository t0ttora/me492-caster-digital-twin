#!/usr/bin/env python3
"""Render case charts and image panels directly from the retained measurement files."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np
from PIL import Image

INK = "#20242a"
MUTED = "#565e67"
BLUE = "#196986"
TEAL = "#368674"
RED = "#bd5140"
AMBER = "#b48038"
GRAY = "#a6adb3"
LIGHT = "#f2f4f5"
LABELS = {
    "clean": "Clean reference", "two_ids": "Two native IDs",
    "duplicate_ids": "Two copies of ID 01", "prepatch_control": "Original prepatch frame",
    "mild_glare": "Mild glare", "hotspot": "Strong glare", "reflection": "Reflection streak",
    "exposure_mild": "Mild exposure", "exposure_clip": "Clipped exposure",
    "exposure_whiteout": "Whiteout", "low_light": "Low light",
    "motion_blur": "43 px motion blur", "low_light_motion_blur": "Low light + blur",
    "near_full_view": "Near full view", "occlusion": "Partial occlusion",
    "edge_crop": "Replay edge crop", "native_near_edge": "Native edge crop",
}
PANELS = {
    "Q09": ("two_ids", "duplicate_ids"),
    "Q10": ("clean", "mild_glare", "hotspot", "reflection", "exposure_clip", "exposure_whiteout"),
    "Q11": ("clean", "low_light", "motion_blur", "low_light_motion_blur"),
    "Q12": ("near_full_view", "occlusion", "edge_crop", "native_near_edge"),
}


def style():
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "axes.labelcolor": MUTED,
                         "axes.titlecolor": INK, "text.color": INK, "axes.edgecolor": GRAY,
                         "axes.spines.top": False, "axes.spines.right": False,
                         "xtick.color": MUTED, "ytick.color": MUTED,
                         "figure.facecolor": "#ffffff", "savefig.facecolor": "#ffffff"})


def gallery(case, rows, fixture_root, manifest, output):
    variants = PANELS[case]
    cols = 2 if len(variants) <= 4 else 3
    fig, axes = plt.subplots(int(np.ceil(len(variants)/cols)), cols, squeeze=False,
                            figsize=(cols*4.0, int(np.ceil(len(variants)/cols))*3.45),
                            layout="constrained")
    fig.suptitle(f"{case} · Camera frames delivered in replay", x=0.018, ha="left",
                 fontsize=16, fontweight="bold")
    for ax, tag in zip(axes.flat, variants):
        subset = [row for row in rows if row["variant"] == tag]
        selected = next((r for r in subset if r["observed"] and r["reason"] not in
                         ("stale_at_publish", "stale_frame")), subset[0] if subset else None)
        index = selected["fixture_frame_index"] if selected else 0
        spec = manifest["fixtures"][tag]["frames"][index]
        with Image.open(fixture_root / spec["path"]) as src:
            ax.imshow(src.convert("RGB"))
        ax.set_xlim(-0.5, 639.5)
        ax.set_ylim(479.5, -0.5)
        ax.axis("off")
        subline = "fixture reference"
        if selected:
            subline = selected["reason"].replace("_", " ").replace("no qr", "no QR")
            subline += f" · frame {selected['frame_sequence']}"
            obs = selected.get("observation")
            if obs:
                for c in obs["candidates"]:
                    points = np.array(c["corners_px"])
                    color = "#14a984" if c["id"] == obs["id"] and obs["valid"] else "#ee865a"
                    ax.add_patch(Polygon(points, fill=False, color=color, linewidth=1.7))
                    ax.text(points[:, 0].mean(), points[:, 1].min()-9,
                            c["id"].replace("JANUS-DOCK-", "ID ") if c["id"] else "undecoded",
                            color="white", size=8, ha="center", va="bottom",
                            bbox={"facecolor": INK, "edgecolor": "none", "pad": 2})
        if case == "Q12" and spec.get("geometric_corners_px"):
            points = np.array(spec["geometric_corners_px"])
            flags = spec["geometric_corners_visible"]
            ax.add_patch(Polygon(points, fill=False, color="#fa815f", linewidth=1.5,
                                 linestyle="--", alpha=.8))
            for point, visible in zip(points, flags):
                if visible:
                    ax.plot(*point, "o", color="#fa815f", ms=4)
            subline += f" · {spec['geometric_visible_corner_count']}/4 geometric corners"
        ax.set_title(f"{LABELS[tag]}\n{subline}", loc="left", fontsize=10, pad=7)
    fig.savefig(output, dpi=170)
    plt.close(fig)


def outcomes(case_data, path):
    trials = case_data["trials"]
    y = np.arange(len(trials))
    fig, ax = plt.subplots(figsize=(8.9, 1.35 + .45*len(trials)), layout="constrained")
    fig.suptitle(f"{case_data['case']} · Disturbance-window outcomes", fontsize=15,
                 x=.02, ha="left", fontweight="bold")
    passed = np.asarray([t["disturbance"]["valid_frames"] for t in trials])
    invalid = np.asarray([t["disturbance"]["observed_frames"] for t in trials]) - passed
    missed = np.asarray([t["disturbance"]["unobserved_frames"] for t in trials])
    for values, left, color, label in ((passed, 0, TEAL, "Valid"),
            (invalid, passed, RED, "Invalid"), (missed, passed+invalid, GRAY, "No observer output")):
        bars = ax.barh(y, values, left=left, color=color, height=.6, label=label)
        for bar, value in zip(bars, values):
            if value:
                ax.text(bar.get_x()+bar.get_width()/2, bar.get_y()+bar.get_height()/2,
                        str(value), color="white" if color != GRAY else INK, weight="bold",
                        va="center", ha="center", size=9)
    labels = [LABELS[t["stimulus"]] + (f" · {int(t['injected_delay_s']*1000)} ms late"
               if t["injected_delay_s"] else "") for t in trials]
    if len(labels) != len(set(labels)):
        labels = [f"{label} · trial {t['trial']+1}" if labels.count(label) > 1 else label
                  for label, t in zip(labels, trials)]
    ax.set_yticks(y, labels)
    ax.invert_yaxis()
    ax.set_xlabel("Published replay frames")
    ax.set_xticks(np.arange(0, int((passed+invalid+missed).max())+1, 2))
    ax.legend(frameon=False, ncol=3, loc="upper center", bbox_to_anchor=(.5, -.23))
    fig.savefig(path, dpi=170)
    plt.close(fig)


def stop_chart(case, trials, trace, origin, frames, path):
    chosen = [t for t in trials if t["phase_results_classification"] == "required rejection"]
    if not chosen:
        return
    cols = min(len(chosen), 3)
    nrows = int(np.ceil(len(chosen)/cols))
    fig, grid = plt.subplots(nrows, cols, squeeze=False, sharex=True, sharey=True,
                            figsize=(4.15*cols, 3.15*nrows), layout="constrained")
    fig.suptitle(f"{case} · Guard output and Gazebo body speed", fontsize=16,
                 x=.015, ha="left", fontweight="bold")
    legend = {}
    for ax, trial in zip(grid.flat, chosen):
        origin_trial = origin + trial["disturbance_s"]
        data = [r for r in trace if r["clock_s"] is not None and
                -.36 <= r["clock_s"]-origin_trial <= 1.22]
        # Ground-truth position/velocity times come from the Gazebo message header.
        for key, time_key, value_key, name, color in (
                ("request", "now_s", "v_mps", "Request", GRAY),
                ("guard_output", "clock_s", "v_mps", "Guard output", BLUE),
                ("ground_truth", "time_s", "planar_speed_mps", "Body speed", TEAL)):
            group = sorted((r for r in data if r["kind"] == key), key=lambda r: r[time_key])
            x = [r[time_key]-origin_trial for r in group]
            y = [r[value_key] for r in group]
            if key == "ground_truth":
                line, = ax.plot(x, y, lw=2, color=color, label=name)
            else:
                line, = ax.step(x, y, where="post", lw=1.4, color=color, label=name)
            legend[name] = line
        ax.axvspan(0, 1.20, color=LIGHT, zorder=-4)
        ax.axhline(.02, color=GRAY, linewidth=1, linestyle="--")
        loss = trial["loss_and_stop"]
        meas = loss["body_stop"]
        if meas:
            ax.axvline(meas["settled_sim_s"]-origin_trial, color=RED, lw=1, ls=":")
            title = f"{LABELS[trial['stimulus']]} · {int(loss['capture_to_guard_zero_sim_ms'])} ms to zero"
        else:
            title = f"{LABELS[trial['stimulus']]} · stop unmeasured"
        if trial["injected_delay_s"]:
            title = f"Low light + stale delivery · {int(loss['capture_to_guard_zero_sim_ms'] or 0)} ms to zero"
        timing_rows = [f for f in frames if f["observed"] and f["trial"] == trial["trial"]
                       and f["phase"] == "disturbance"]
        timing_count = sum(f["valid"] for f in timing_rows)
        ax.set_title(f"{title}\n{len(timing_rows)} observations · {timing_count} valid",
                     loc="left", fontsize=9.5)
        ax.set_xlim(-.36, 1.22)
        ax.set_ylim(-.005, .101)
        ax.grid(axis="y", alpha=.3)
    for ax in list(grid.flat)[len(chosen):]:
        ax.axis("off")
    for ax in grid[-1, :]:
        if ax.axison:
            ax.set_xlabel("Simulation seconds from disturbance onset")
    for ax in grid[:, 0]:
        ax.set_ylabel("Linear speed (m/s)")
    if legend:
        fig.legend(list(legend.values()), list(legend.keys()), loc="outside lower center",
                   ncol=3, frameon=False)
    fig.savefig(path, dpi=170)
    plt.close(fig)


def quality_chart(case, trials, frames, path):
    if case != "Q11":
        return
    tags = ("clean", "low_light", "motion_blur", "low_light_motion_blur", "late")
    colors = (GRAY, TEAL, BLUE, RED, AMBER)
    late = {t["trial"] for t in trials if t["injected_delay_s"]}
    by_tag = [[r for r in frames if r["observed"] and r["variant"] == tag
               and not (r["trial"] in late and r["phase"] == "disturbance")] for tag in tags[:-1]]
    by_tag.append([r for r in frames if r["observed"] and r["trial"] in late and r["phase"] == "disturbance"])
    labels = [LABELS[tag] for tag in tags[:-1]] + ["Low light + late delivery"]
    fig, axs = plt.subplots(1, 3, figsize=(12.4, 3.65), layout="constrained")
    fig.suptitle("Q11 · Measured quality and frame age", x=.015, ha="left",
                 fontsize=16, fontweight="bold")
    for ax, metric, title in zip(axs, ("roi_luma_span_p95_p05", "roi_laplacian_variance",
                                       "capture_to_publish_sim_ms"),
                                  ("ROI luminance span (8-bit DN)", "ROI Laplacian variance (DN²)",
                                   "Capture to publication (sim ms)")):
        for i, (group, color) in enumerate(zip(by_tag, colors)):
            values = [r[metric] for r in group]
            if not values:
                continue
            lo, middle, hi = np.percentile(values, [5, 50, 95])
            ax.hlines(i, lo, hi, color=color, linewidth=2)
            ax.plot(middle, i, "o", color=color, ms=5)
        ax.set_title(title, loc="left", fontsize=10.5)
        ax.set_yticks(range(len(tags)), labels if ax is axs[0] else [""]*len(tags))
        ax.invert_yaxis()
        ax.grid(axis="x", alpha=.27)
    axs[-1].axvline(200, color=RED, linewidth=1, ls="--")
    axs[-1].set_xlabel("Dot shows median. Bar spans P05 to P95.")
    fig.savefig(path, dpi=170)
    plt.close(fig)


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("fixtures", type=Path)
    cli.add_argument("analysis", type=Path)
    cli.add_argument("recording", type=Path)
    args = cli.parse_args()
    style()
    summary = json.loads((args.analysis/"qualification.json").read_text())
    manifest = json.loads((args.fixtures/"manifest.json").read_text())
    trace = [json.loads(line) for line in (args.recording/"trace.jsonl").read_text().splitlines()]
    for case, result in summary["cases"].items():
        case_dir = args.analysis/case
        frames = [json.loads(line) for line in (case_dir/"frames.jsonl").read_text().splitlines()]
        gallery(case, frames, args.fixtures, manifest, case_dir/"camera-frames.png")
        outcomes(result, case_dir/"outcomes.png")
        stop_chart(case, result["trials"], trace, summary["simulation_origin_s"],
                   frames, case_dir/"stop-trace.png")
        quality_chart(case, result["trials"], frames, case_dir/"quality-latency.png")


if __name__ == "__main__":
    main()
