#!/usr/bin/env python3
"""Run JANUS offline fixtures and emit auditable experiment artifacts."""
import argparse
import csv
from hashlib import file_digest, sha256
import json
from pathlib import Path
import platform
import re
import shutil
import sys
from time import perf_counter_ns

import cv2
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "src/arche_experiments"))

from arche_experiments.commands import VELOCITY, load_commands
from arche_experiments.fixtures import HEIGHT, centering_fixture, guard_fixture, qr_fixtures, timing_fixture
from arche_experiments.replay import ideal_velocity_replay
from arche_experiments.trace_metrics import measure_trace
from arche_experiments.vision import Decoder, Observation, TARGET_ID

MODEL = ROOT / "src/arche_sim"
CONFIG = MODEL / "config/robot_config.json"
CASES = tuple(f"{group}{index:02d}" for group, count in (("M", 12), ("Q", 20), ("O", 8))
              for index in range(1, count + 1))
GRID_S = 0.02


def checksum(path):
    with path.open("rb") as stream:
        return file_digest(stream, "sha256").hexdigest()


def write_json(path, payload):
    path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n")


def write_csv(path, rows):
    if not rows:
        return
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fields)
        writer.writeheader()
        writer.writerows(rows)


def draw_xy(path, rows):
    xs = np.array([row["x_m"] for row in rows])
    ys = np.array([row["y_m"] for row in rows])
    span = max(float(np.ptp(xs)), float(np.ptp(ys)), 0.01)
    # Keep plot axes at the same scale. SVG y points down.
    px = (xs - (xs.max() + xs.min()) / 2) / span * 240 + 190
    py = -(ys - (ys.max() + ys.min()) / 2) / span * 240 + 160
    stride = max(1, len(px) // 1000)
    points = " ".join(f"{x:.2f},{y:.2f}" for x, y in zip(px[::stride], py[::stride]))
    path.write_text(f"""<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 380 330">
  <title>XY trajectory in metres</title><rect width="380" height="330" fill="#f4f7fa"/>
  <polyline points="{points}" fill="none" stroke="#225a79" stroke-width="3"/>
  <circle cx="{px[0]:.2f}" cy="{py[0]:.2f}" r="5" fill="#006447"/>
  <circle cx="{px[-1]:.2f}" cy="{py[-1]:.2f}" r="3" fill="#a3210d"/>
  <path d="M 20 305 h 60" stroke="black"/><text x="20" y="323" font-size="12">{span / 4:.4g} m</text>
</svg>
""")


def run_file(base, case, variant, scope, command=None):
    run_id = f"{case}-gazebo-{variant}"
    directory = base / run_id
    directory.mkdir()
    manifest = {"run_id": run_id, "case_id": case, "variant": variant,
                "evidence_scope": scope, "kind": "new_execution", "seed": None,
                "software": {"platform": platform.platform(), "python": sys.version.split()[0],
                             "opencv": cv2.__version__, "numpy": np.__version__},
                "solver_step_s": None, "log_sample_period_s": None,
                "model_hashes": {str(p.relative_to(ROOT)): checksum(p)
                                 for p in sorted(MODEL.rglob("*"))
                                 if p.is_file() and p.suffix in (".sdf", ".dae", ".json", ".yaml", ".config", ".py")},
                "command_sha256": checksum(command) if command else None,
                "command_input": str(command.resolve()) if command else None,
                "case_acceptance": "blocked_pending_native_case_run",
                "termination_reason": "offline_check_finished"}
    manifest["model_sha256"] = sha256(
        json.dumps(manifest["model_hashes"], sort_keys=True).encode()).hexdigest()
    return directory, manifest


def finish(directory, manifest, metrics, passed, evidence, summaries):
    manifest.update(check_status="passed" if passed else "failed", evidence=evidence)
    write_json(directory / "metrics.json", metrics)
    write_json(directory / "manifest.json", manifest)
    summaries.append({key: manifest[key] for key in
                      ("case_id", "run_id", "variant", "evidence_scope", "check_status",
                       "case_acceptance", "termination_reason")} |
                     {"manifest": f"{directory.name}/manifest.json", "metrics": f"{directory.name}/metrics.json"})


def replay_command(out, case, variant, input_path, summaries, initial=None, origin_file=None,
                   semantics="fixture", upstream=None):
    directory, manifest = run_file(out, case, variant, "offline_command_replay", input_path)
    shutil.copyfile(input_path, directory / "commands.csv")
    manifest["command_semantics"] = semantics
    if origin_file:
        shutil.copyfile(origin_file, directory / "input_manifest.json")
        manifest["input_manifest_sha256"] = checksum(directory / "input_manifest.json")
    manifest["command_sha256"] = checksum(directory / "commands.csv")
    if upstream is not None:
        manifest["upstream_status"] = upstream.get("status")
        manifest["upstream_model_sha256"] = upstream.get("model_sha256")
        manifest["upstream_initial_solver_state"] = upstream.get("initial_solver_state")
        expected_hash = upstream.get("files", {}).get(input_path.name)
        if expected_hash != manifest["command_sha256"]:
            manifest["termination_reason"] = "upstream_command_hash_mismatch"
            finish(directory, manifest, {"expected_sha256": expected_hash}, False,
                   ["commands.csv", "metrics.json"], summaries)
            return
    try:
        series = load_commands(directory / "commands.csv")
    except ValueError as error:
        manifest["termination_reason"] = "invalid_command_input"
        finish(directory, manifest, {"error": str(error)}, False, ["commands.csv", "metrics.json"], summaries)
        return
    timed = perf_counter_ns()
    _, first_column, second_column = series.schema
    metrics = {"command_rows": len(series.time), "command_duration_s": series.time[-1],
               f"peak_abs_{first_column}": max(abs(v[0]) for v in series.values),
               f"peak_abs_{second_column}": max(abs(v[1]) for v in series.values)}
    if series.schema != VELOCITY:
        manifest["evidence_scope"] = "offline_torque_schema"
        manifest["termination_reason"] = "torque_schema_verified_effort_plant_unavailable"
        finish(directory, manifest, metrics, True, ["commands.csv", "metrics.json"], summaries)
        return
    config = json.loads(CONFIG.read_text())
    origin = (0.0, 0.0, 0.0)
    if isinstance(initial, dict) and all(key in initial for key in ("x_m", "y_m", "yaw_rad")):
        origin = tuple(float(initial[key]) for key in ("x_m", "y_m", "yaw_rad"))
    manifest["initial_axle_pose_x_y_yaw"] = origin
    manifest["log_sample_period_s"] = GRID_S
    manifest["log_sample_period_note"] = "additional rows exactly at ZOH command edges"
    rows = list(ideal_velocity_replay(series, GRID_S, config["drive_radius_m"], config["drive_track_m"], origin))
    metrics.update(wall_replay_ms=(perf_counter_ns() - timed) / 1e6,
                   grid_samples=len(rows), ideal_path_length_m=rows[-1]["integrated_abs_distance_m"],
                   ideal_final_pose_x_y_yaw=[rows[-1][axis] for axis in ("x_m", "y_m", "yaw_rad")],
                   wheel_rate_is_kinematic=True, caster_contact_force=None, applied_axle_effort=None)
    write_csv(directory / "results.csv", rows)
    draw_xy(directory / "trajectory.svg", rows)
    finish(directory, manifest, metrics, True,
           ["commands.csv", "results.csv", "trajectory.svg", "metrics.json"], summaries)


def replay_image(out, fixture, summaries, repeats, decoder, marker_path):
    case, variant, pixels, target_u, expected_valid, reason, turn_sign = fixture
    directory, manifest = run_file(out, case, variant, "synthetic_image_decode")
    png = directory / "frame.png"
    Image.fromarray(pixels).save(png)
    pixels = cv2.imread(str(png), cv2.IMREAD_GRAYSCALE)
    request = Observation(0.0, 0.0, pixels.shape[1], pixels.shape[0], TARGET_ID, target_u)
    decoder.decode(pixels, request)
    timing_rows = []
    for repeat in range(1, repeats + 1):
        start = perf_counter_ns()
        observation = decoder.decode(pixels, request)
        timing_rows.append({"repeat": repeat, "decode_ms": (perf_counter_ns() - start) / 1e6,
                            "valid": observation.valid, "reason": observation.reason,
                            "selected_id": observation.id, "candidate_count": len(observation.candidates)})
    durations = [row["decode_ms"] for row in timing_rows]
    error = observation.error
    actual_direction = -(int(np.sign(error))) if error is not None else None
    checks = {"validity": observation.valid == expected_valid,
              "reason": reason is None or observation.reason == reason,
              "yaw_sign": turn_sign is None or actual_direction == turn_sign,
              "accepted_id": not observation.valid or observation.id == TARGET_ID,
              "consistent_repeats": all(
                  all(row[key] == timing_rows[-1][key] for key in
                      ("valid", "reason", "selected_id", "candidate_count"))
                  for row in timing_rows)}
    if case == "Q09":
        checks["two_candidates"] = len(observation.candidates) == 2
    if variant in ("nominal", "rotated_shifted_target"):
        checks["inside_2_percent"] = error is not None and abs(error) <= 0.02
    if variant == "rotated_shifted_target" and observation.corners_px:
        first_corner = observation.corners_px[0]
        checks["physical_top_left_after_rotation"] = first_corner[0] > target_u and first_corner[1] < HEIGHT / 2
    manifest.update(fixture_sha256=checksum(png), marker_source_sha256=checksum(marker_path) if marker_path else None,
                    fixture_schedule="single PNG at capture=arrival=0; repeated decoding for wall timing")
    write_json(directory / "results.json", observation.as_row())
    write_csv(directory / "timing.csv", timing_rows)
    metrics = {"valid": observation.valid, "candidate_count": len(observation.candidates),
               "decode_repeat_count": repeats, "decode_p50_ms": float(np.percentile(durations, 50)),
               "decode_p95_ms": float(np.percentile(durations, 95)), "decode_max_ms": max(durations),
               "normalized_error": error, "reason": observation.reason, "checks": checks}
    finish(directory, manifest, metrics, all(checks.values()),
           ["frame.png", "results.json", "timing.csv", "metrics.json"], summaries)


def log_message_check(out, case, variant, fixture, summaries):
    directory, manifest = run_file(out, case, variant, "synthetic_message_replay")
    rows = list(fixture)
    write_csv(directory / "results.csv", rows)
    passed = sum(row["passed"] for row in rows)
    manifest["fixture_schedule"] = "source fixtures.py and event times captured in results.csv"
    finish(directory, manifest, {"message_rows": len(rows), "matched_rows": passed},
           passed == len(rows), ["results.csv", "metrics.json"], summaries)


def python_inputs(args, summaries):
    if args.python_dir is None:
        return
    for input_manifest in sorted(args.python_dir.rglob("manifest.json")):
        provenance = json.loads(input_manifest.read_text())
        upstream_id = provenance.get("run_id", input_manifest.parent.name)
        if not isinstance(upstream_id, str) or not re.fullmatch(r"[MQO]\d{2}-python-[a-zA-Z0-9_-]+", upstream_id):
            continue
        case = upstream_id[:3]
        if case not in CASES:
            continue
        initial = provenance.get("initial_axle_pose", provenance.get("initial_pose"))
        for name, semantics in (
            ("commands.csv", "upstream requested body velocity"),
            ("executed_velocity.csv", "upstream limited wheel-PI reference, expressed as body velocity"),
            ("applied_torque.csv", "upstream applied axle effort; schema validation"),
        ):
            source = input_manifest.parent / name
            if source.is_file():
                replay_command(args.out, case, f"upstream-{upstream_id[11:]}-{source.stem}", source,
                               summaries, initial, input_manifest, semantics, provenance)


def plan_rows(path):
    if path is None:
        return {}
    lines = re.findall(r"^\| ([MQO]\d{2}) \| (.*?) \| (P[012]) \| (.*?) \|$", path.read_text(), flags=re.M)
    return {case: (title, priority, requirement) for case, title, priority, requirement in lines}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", action="store_true")
    parser.add_argument("--fixtures", type=Path)
    parser.add_argument("--plan", type=Path)
    parser.add_argument("--contract", type=Path)
    parser.add_argument("--python-dir", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--repeats", type=int, default=15)
    parser.add_argument("--case", choices=CASES)
    parser.add_argument("--commands", type=Path)
    parser.add_argument("--trace", type=Path)
    parser.add_argument("--native-run-id")
    args = parser.parse_args()
    if args.repeats < 1 or (args.suite and not args.fixtures):
        parser.error("suite needs --fixtures and a positive repeat count")
    if args.commands and not args.case:
        parser.error("--commands needs --case")
    if args.trace and not (args.commands and args.native_run_id):
        parser.error("--trace needs --commands and --native-run-id")
    if not (args.suite or args.commands or args.python_dir):
        parser.error("select --suite, --commands or --python-dir")
    args.out.mkdir(parents=True, exist_ok=False)
    summaries = []
    if args.suite:
        for kind in ("velocity", "torque"):
            replay_command(args.out, "M01", f"zero-{kind}", args.fixtures / f"M01-zero-{kind}.csv",
                           summaries, origin_file=args.fixtures / "M01-zero-fixture.json")
        decoder = Decoder()
        for fixture in qr_fixtures():
            replay_image(args.out, fixture, summaries, args.repeats, decoder, None)
        output_marker = args.out / "station_texture.png"
        import importlib.util
        loader = importlib.util.spec_from_file_location("make_marker", MODEL / "scripts/make_marker.py")
        make_marker = importlib.util.module_from_spec(loader)
        loader.loader.exec_module(make_marker)
        make_marker.write_marker(CONFIG, output_marker)
        station = cv2.imread(str(output_marker), cv2.IMREAD_GRAYSCALE)
        replay_image(args.out, ("Q01", "station-texture", station, station.shape[1] / 2,
                               True, "accepted", None), summaries, args.repeats, decoder, output_marker)
        log_message_check(args.out, "Q13", "timestamps", timing_fixture(), summaries)
        log_message_check(args.out, "Q13", "centering-stream-cut", centering_fixture(), summaries)
        for case, variants in (("O04", ("command_timeout", "operator_abort")),
                               ("O05", ("odom_timeout", "odom_regression", "clock_stall", "clock_reset"))):
            for variant in variants:
                log_message_check(args.out, case, variant, guard_fixture(variant), summaries)
    python_inputs(args, summaries)
    if args.commands:
        replay_command(args.out, args.case, "provided-command", args.commands, summaries,
                       semantics="user-supplied; consult originating manifest")
    if args.trace:
        directory, manifest = run_file(args.out, args.case, "bag-trace", "gazebo_bag_export_input", args.commands)
        manifest.update(native_run_id=args.native_run_id, trace_path=str(args.trace.resolve()),
                        trace_sha256=checksum(args.trace), log_sample_period_s=GRID_S)
        metrics, rows = measure_trace(args.trace, args.native_run_id, manifest["command_sha256"], GRID_S)
        if rows:
            write_csv(directory / "results.csv", rows)
            draw_xy(directory / "trajectory.svg", rows)
        manifest["case_acceptance"] = "requires_case_specific_evaluation"
        manifest["termination_reason"] = metrics.get("reason", "trace_overlap_measured")
        finish(directory, manifest, metrics, bool(rows),
               ["results.csv", "trajectory.svg", "metrics.json"] if rows else ["metrics.json"], summaries)
    planned = plan_rows(args.plan)
    if args.plan and set(planned) != set(CASES):
        raise ValueError("plan does not match the 40 contract case IDs")
    cases = []
    for case in CASES:
        scoped = [row for row in summaries if row["case_id"] == case]
        title, priority, requirement = planned.get(case, ("", "", ""))
        cases.append({"case_id": case, "case": title, "priority": priority,
                      "status": "partial" if scoped else "blocked",
                      "checks_passed": sum(row["check_status"] == "passed" for row in scoped),
                      "checks_failed": sum(row["check_status"] == "failed" for row in scoped),
                      "native_acceptance": "requires_case_evaluation" if args.trace and case == args.case
                      else "blocked",
                      "required_evidence": requirement,
                      "evidence_run_ids": "|".join(row["run_id"] for row in scoped)})
    write_csv(args.out / "case_results.csv", cases)
    write_csv(args.out / "runs.csv", summaries)
    write_json(args.out / "manifest.json", {
        "arguments": sys.argv[1:], "reproduce_from": str(ROOT),
        "plan_sha256": checksum(args.plan) if args.plan else None,
        "contract_sha256": checksum(args.contract) if args.contract else None,
        "cases": len(cases), "run_count": len(summaries), "runs": summaries,
        "scopes": {"native_launch": "blocked_without_linux", "physical_trial": "blocked",
                   "synthetic_images": "real decoder on deterministic inputs",
                   "command_replay": "analytic ideal no-slip integration driven by held commands"},
    })
    failures = [row["run_id"] for row in summaries if row["check_status"] == "failed"]
    print(f"{len(summaries) - len(failures)}/{len(summaries)} offline checks passed; {len(cases)} case rows")
    if failures:
        print("Failed checks: " + ", ".join(failures))
    return bool(failures)


if __name__ == "__main__":
    raise SystemExit(main())
