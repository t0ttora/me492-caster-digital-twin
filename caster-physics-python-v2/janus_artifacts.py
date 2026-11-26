"""Persist tabular evidence, diagnostic plots and standalone motion playback."""
from dataclasses import asdict
from datetime import datetime, timezone
from hashlib import sha256
import csv
import json
import platform
import sys

import numpy as np

from qr_camera import DOCK_ID
from qr_docking import Dock
from scenarios import P0
from twin import ROOT, V1


def digest(path):
    return sha256(path.read_bytes()).hexdigest()


def atomic_json(path, value):
    temporary = path.with_suffix(path.suffix+".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    temporary.replace(path)


def csv_file(path, columns, rows):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        writer.writerows(rows)


def dict_csv(path, rows, columns=None):
    keys = list(columns if columns is not None else rows[0])
    csv_file(path, keys, ([row.get(key) for key in keys] for row in rows))


def overview(path, run_id, rows, events):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    def c(field):
        return [row[field] for row in rows]

    time = c("time_s")
    fig, axes = plt.subplots(3, 2, figsize=(11, 9), layout="constrained")
    fig.suptitle(run_id+"  •  V2.3 synthetic run")
    is_dock = any(row["dock_geometry_error_m"] is not None for row in rows)
    plot = axes[0, 0]
    if is_dock:
        plot.scatter(c("ref_x_m")[-1], c("ref_y_m")[-1], color="#9b7d4b", marker="x", label="Dock target")
    else:
        plot.plot(c("ref_x_m"), c("ref_y_m"), "--", label="Reference")
    x, y = np.asarray(c("x_m")), np.asarray(c("y_m"))
    plot.plot(x, y, label="Axle path")
    stride = max(1, len(rows)//16)
    yaw = np.asarray(c("yaw_rad"))[::stride]
    span = max(.12, np.ptp(x), np.ptp(y))
    plot.quiver(x[::stride], y[::stride], np.cos(yaw), np.sin(yaw),
                angles="xy", scale_units="xy", scale=20/span, color="#137a7f", width=.004)
    plot.set(xlabel="x [m]", ylabel="y [m]")
    plot.set_aspect("equal", adjustable="box")
    x_extent, y_extent = np.r_[x, c("ref_x_m")], np.r_[y, c("ref_y_m")]
    x_center, y_center = (x_extent.min()+x_extent.max())/2, (y_extent.min()+y_extent.max())/2
    half_x = max(np.ptp(x_extent)/2+.08*span, .22*span)
    half_y = max(np.ptp(y_extent)/2+.08*span, .55*half_x)
    plot.set_xlim(x_center-half_x, x_center+half_x)
    plot.set_ylim(y_center-half_y, y_center+half_y)
    ax = axes[0, 1]
    error = c("cross_track_error_m") if rows[0]["cross_track_error_m"] is not None else c("pose_error_m")
    ax.plot(time, error, label="Cross-track [m]" if rows[0]["cross_track_error_m"] is not None else "Target distance [m]")
    ax.plot(time, np.abs(c("yaw_error_rad")), label="Absolute yaw error [rad]")
    ax = axes[1, 0]
    ax.plot(time, c("caster_angle_body_rad"), label="Caster angle [rad]")
    ax.plot(time, c("caster_relative_rate_radps"), label="Caster swivel [rad/s]")
    ax = axes[1, 1]
    ax.plot(time, c("left_wheel_rate_radps"), label="Left wheel [rad/s]")
    ax.plot(time, c("right_wheel_rate_radps"), label="Right wheel [rad/s]")
    effort = ax.twinx()
    effort.plot(time, c("left_drive_torque_Nm"), "--", alpha=.4, color="C0")
    effort.plot(time, c("right_drive_torque_Nm"), "--", alpha=.4, color="C1")
    effort.set_ylabel("Applied torque [N m], dashed")
    ax = axes[2, 0]
    ax.plot(time, c("requested_v_mps"), label="Requested v [m/s]")
    ax.plot(time, c("axle_v_mps"), label="Measured axle v [m/s]")
    ax.plot(time, c("limited_yaw_rate_radps"), label="Limited yaw [rad/s]", alpha=.7)
    ax = axes[2, 1]
    if is_dock:
        ax.plot(time, c("qr_error"), label="QR horizontal error")
        ax.axhspan(-.02, .02, color="#137a7f", alpha=.12, label="Inner band")
        for band in (-.04, .04):
            ax.axhline(band, color="#137a7f", alpha=.35, linestyle="--")
    else:
        ax.plot(time, c("motor_work_J"), label="Mechanical axle work [J]")
        ax.plot(time, c("dissipated_energy_J"), label="Dissipated energy [J]")
    for axis in axes.flat:
        axis.legend(loc="best", fontsize=7)
        axis.grid(alpha=.18)
        if axis is not axes[0, 0]:
            axis.set_xlabel("Time [s]")
    labels = {"gate_accepted", "lost", "aligned", "contact", "success", "timeout", "abort",
              "command_timeout", "operator_cancel", "pedestrian", "authorized_resume", "stop"}
    marked = [event["time_s"] for event in events if event["event"] in labels]
    for axis in axes.flat[1:]:
        for t in sorted(set(marked)):
            axis.axvline(t, alpha=.23, color="#595959", linewidth=.8)
    fig.savefig(path, dpi=145)
    plt.close(fig)


def motion(path, run_id, rows, geometry):
    keys = ("time_s", "x_m", "y_m", "yaw_rad", "caster_angle_body_rad",
            "ref_x_m", "ref_y_m", "phase", "reason", "qr_error", "requested_v_mps")
    frames = [[round(row[k], 6) if isinstance(row[k], (int, float)) else row[k] for k in keys] for row in rows[::4]]
    if frames[-1][0] != rows[-1]["time_s"]:
        frames.append([rows[-1][k] for k in keys])
    info = json.dumps({"frames": frames, "track": geometry["drive_track_m"], "trail": geometry["caster_trail_m"],
                       "caster": geometry["caster_pivot_m"][:2], "chassis": geometry["chassis_size_m"][:2]},
                      allow_nan=False)
    template = (ROOT / "janus_motion.html").read_text(encoding="utf-8")
    path.write_text(template.replace("__RUN__", run_id).replace("__DATA__", info), encoding="utf-8")


def observation_rows(frames, camera):
    width, height = camera["width_px"], camera["height_px"]
    for frame in frames:
        if not frame.candidates:
            yield [frame.capture_time_s, frame.arrival_time_s, width, height, "", "", "", False, 0.,
                   False, "missing", frame.source]+[""]*8
        for candidate in frame.candidates:
            corners = candidate.corners_px
            center = candidate.center_px or ("", "")
            valid = candidate.id == DOCK_ID and candidate.visible and candidate.quality >= .5
            reason = "valid_candidate" if valid else ("wrong_id" if candidate.id != DOCK_ID else "not_visible")
            yield [frame.capture_time_s, frame.arrival_time_s, width, height, candidate.id, *center,
                   candidate.visible, candidate.quality, valid, reason, frame.source]+(
                       [v for point in corners for v in point] if corners else [""]*8)


def write_run(output, fixture, run, metrics, baseline):
    run_id = run["rows"][0]["run_id"]
    directory = output/run_id
    staging = output/("."+run_id+".writing")
    staging.mkdir()
    rows = run["rows"]
    dict_csv(staging/"results.csv", rows)
    dict_csv(staging/"events.csv", run["events"], ("time_s", "event", "reason"))
    csv_file(staging/"commands.csv", ("time_s", "v_mps", "yaw_rate_radps"),
             ((r["time_s"], r["requested_v_mps"], r["requested_yaw_rate_radps"]) for r in rows))
    csv_file(staging/"executed_velocity.csv", ("time_s", "v_mps", "yaw_rate_radps"),
             ((r["time_s"], r["limited_v_mps"], r["limited_yaw_rate_radps"]) for r in rows))
    csv_file(staging/"applied_torque.csv", ("time_s", "tau_left_Nm", "tau_right_Nm"),
             ((r["time_s"], r["left_drive_torque_Nm"], r["right_drive_torque_Nm"]) for r in rows))
    camera = run["configuration"]["camera"]
    if camera:
        columns = ["capture_time_s", "arrival_time_s", "width_px", "height_px", "id", "center_u_px",
                   "center_v_px", "visible", "quality", "candidate_valid", "candidate_reason", "source"]
        columns.extend(f"{corner}_{axis}_px" for corner in ("tl","tr","br","bl") for axis in ("u","v"))
        csv_file(staging/"qr_observations.csv", columns, observation_rows(run["frames"], camera))
    atomic_json(staging/"metrics.json", metrics)
    atomic_json(staging/"robot_config.json", run["configuration"])
    atomic_json(staging/"scenario.json", asdict(fixture))
    overview(staging/"overview.png", run_id, rows, run["events"])
    motion(staging/"motion.html", run_id, rows, run["configuration"]["parameters"]["geometry"])
    status = "failed" if not all(metrics["checks"].values()) else ("partial" if fixture.hardware_scope else "passed")
    scopes = ["synthetic_plant"]+["synthetic_projection", "geometry_proxy"] if camera else ["synthetic_plant"]
    if fixture.hardware_scope and "geometry proxy" in fixture.hardware_scope and "geometry_proxy" not in scopes:
        scopes.append("geometry_proxy")
    dynamics = run["configuration"]["parameters"]["dynamics"]
    source_paths = list(ROOT.glob("*.py"))+[ROOT/"janus_motion.html"]+list((ROOT/"parameters").glob("*.json"))
    model_files = [*sorted((V1/"src/caster_physics").rglob("*.py")), ROOT/"twin.py"]
    model_sources = {str(p.relative_to(ROOT.parent)): digest(p) for p in model_files}
    model_hash = sha256(json.dumps(model_sources, sort_keys=True).encode()).hexdigest()
    manifest = {
        "schema_version": "janus-python-1", "run_id": run_id, "case_id": fixture.case,
        "variant": fixture.variant, "priority": "P0" if fixture.case in P0 else "P1",
        "status": status, "measurement_checks_passed": all(metrics["checks"].values()),
        "execution": "newly_executed", "created_utc": datetime.now(timezone.utc).isoformat(),
        "evidence_scopes": scopes, "physical_trial": False, "fixture_scope_limit": fixture.hardware_scope,
        "software": {"python": platform.python_version(), "numpy": np.__version__,
                     "host": platform.platform(), "plant": "V2.3 with preserved V1 M2 dynamics"},
        "baseline": baseline,
        "source_sha256": {str(p.relative_to(ROOT)): digest(p) for p in sorted(source_paths)},
        "model_sha256": model_hash, "model_sources_sha256": model_sources,
        "command_sha256": digest(staging/"commands.csv"),
        "fixture_event_schedule": fixture.events, "seed": fixture.seed,
        "solver": {"method": "RK4", "step_s": dynamics["solver_step_s"],
                   "command_period_s": dynamics["command_sample_dt_s"],
                   "result_period_s": dynamics["result_sample_dt_s"], "applied_torque_interpolation": "zoh"},
        "initial_axle_pose": dict(zip(("x_m", "y_m", "yaw_rad"), run["initial_axle_pose"])),
        "initial_solver_state": run["initial_solver_state"],
        "termination_reason": run["termination_reason"], "completed_time_s": run["rows"][-1]["time_s"],
        "command_roles": {"commands.csv": "requested body velocity", "executed_velocity.csv": "limited wheel-PI input",
                          "applied_torque.csv": "actual ZOH axle torque; use for effort replay"},
        "frames": {"world_and_body": "x forward, y left, yaw positive CCW; results x/y are axle midpoint; solver x/y are COM",
                   "camera": "optical x right, y down, z forward"},
        "qr": {"station_id": DOCK_ID, "inner_band": Dock.INNER, "outer_band": Dock.OUTER,
               "center_dwell_s": .4, "freshness_s": camera["freshness_s"], "bms_freshness_s": Dock.BMS_FRESHNESS_S,
               "bms_source": "virtual_fixture" if any(e=="virtual_charge" for _,e in fixture.events) else "absent",
               "contact_source": "synthetic_standoff_and_pose_gate",
               "center_timeout_s": Dock.CENTER_TIMEOUT_S, "image_yaw_gain_radps": fixture.qr_yaw_gain,
               "search_sweep": fixture.search_sweep} if camera else None,
        "reproduce_argv": [sys.executable, str(ROOT/"run_study.py"), "--case", fixture.case, "--variant", fixture.variant,
                           "--output-dir", str(output.with_name(output.name+"-repro"))],
        "files": {p.name: digest(p) for p in sorted(staging.iterdir())},
    }
    atomic_json(staging/"manifest.json", manifest)
    staging.rename(directory)
    return manifest
