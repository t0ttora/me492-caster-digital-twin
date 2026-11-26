"""Refresh plots from saved CSV and create a flat handoff measurement table."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from janus_artifacts import atomic_json, dict_csv, digest, motion, overview
from verify_janus import read_csv


def typed(row):
    names = {"run_id", "phase", "reason", "surface", "qr_reject_reason"}
    booleans = {"qr_valid", "aligned", "contact", "charge_confirmed"}
    return {key: (value if key in names else value == "True" if key in booleans else
                  float(value) if value else None) for key, value in row.items()}


def finalize(folder):
    index = json.loads((folder/"case_status.json").read_text())
    if index["state"] != "batch_finished":
        raise ValueError("Finish the run batch before producing the final measurement table")
    table = []
    for record in index["runs"]:
        path = folder / record["manifest"]
        manifest = json.loads(path.read_text())
        directory = path.parent
        metrics = json.loads((directory/"metrics.json").read_text())
        config = json.loads((directory/"robot_config.json").read_text())
        rows = [typed(row) for row in read_csv(directory/"results.csv")]
        events = read_csv(directory/"events.csv")
        for event in events:
            event["time_s"] = float(event["time_s"])
        overview(directory/"overview.new.png", record["run_id"], rows, events)
        motion(directory/"motion.new.html", record["run_id"], rows, config["parameters"]["geometry"])
        for name in ("overview.png", "motion.html"):
            stem, extension = name.split(".")
            (directory/f"{stem}.new.{extension}").replace(directory/name)
            manifest["files"][name] = digest(directory/name)
        base = Path(__file__).resolve().parents[1]
        manifest["rendered_from_results_csv"] = {
            "input_sha256": manifest["files"]["results.csv"],
            "generator_sha256": digest(base/"janus_artifacts.py"),
            "motion_template_sha256": digest(base/"janus_motion.html"),
        }
        atomic_json(path, manifest)
        table.append({
            "run_id": record["run_id"], "case_id": record["case_id"], "status": record["status"],
            "evidence_scopes": ",".join(record["evidence_scopes"]), "manifest": record["manifest"],
            "termination_reason": record["termination_reason"],
            "duration_s": metrics["duration_s"], "gates_accepted": len(metrics["gates"]),
            "last_gate_time_s": metrics["gates"][-1]["time_s"] if metrics["gates"] else None,
            **{key: metrics[key] for key in (
                "scheduled_reference_duration_s", "path_rmse_m", "final_position_error_m", "final_yaw_error_rad",
                "stop_duration_s", "stop_distance_m", "stop_dwell_verified_at_s",
                "peak_torque_Nm", "rms_left_torque_Nm", "rms_right_torque_Nm", "torque_saturated_sample_fraction",
                "caster_lateral_slip_rms_mps", "caster_peak_abs_swivel_rate_radps", "caster_realign_status",
                "caster_realign_10deg_dwell_0_25s_after_motion_s", "mechanical_axle_work_J", "drive_long_slip_rms_mps",
                "qr_first_error", "qr_first_yaw_request_radps", "qr_first_advance_s", "qr_invalid_forward_samples",
                "dock_final_position_error_m", "dock_final_yaw_error_rad", "aligned", "contact", "charge_confirmed",
                "retry_count", "qr_yaw_sign_changes")},
            "minimum_wall_clearance_m": metrics["clearance_geometry_proxy"].get("minimum_wall_clearance_m"),
            "failed_checks": ",".join(key for key, result in metrics["checks"].items() if not result),
        })
    dict_csv(folder/"measurements.csv", table)
    print(f"Exported {len(table)} measurement rows and refreshed their plots", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", type=Path)
    finalize(parser.parse_args().runs)
