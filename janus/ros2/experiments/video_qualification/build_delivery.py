#!/usr/bin/env python3
"""Build the local report and case manifests from the measured replay outputs."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import re
import subprocess


def load(path):
    return json.loads(path.read_text())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def json_write(path, data):
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + "\n")


def select_trials(case, labels):
    return [t for t in case["trials"] if t["stimulus"] in labels]


def span(values, digits=0, unit=""):
    data = [value for value in values if value is not None]
    if not data:
        return "unmeasured"
    limits = (f"{min(data):.{digits}f}", f"{max(data):.{digits}f}")
    return (limits[0] if limits[0] == limits[1] else "–".join(limits)) + unit


def status_detail(case):
    if case["status"] == "passed":
        return {"Q09": "Passed", "Q10": "Passed", "Q11": "Passed",
                "Q12": "Passed for the QR-only stop"}[case["case"]]
    issues = [key.replace("_", " ") for key, value in case["checks"].items() if not value]
    return "Partial · " + ", ".join(issues)


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("lane", type=Path)
    cli.add_argument("analysis", type=Path)
    cli.add_argument("primary", type=Path)
    args = cli.parse_args()
    lane, analysis, primary = (p.resolve() for p in (args.lane, args.analysis, args.primary))
    source = lane / "checkout"
    files = load(analysis / "qualification.json")
    schedule = load(primary / "recording/schedule.json")
    matrix = load(lane/"originals/qualification-matrix.json")
    originals = lane / "originals"
    entries = (matrix["cases"] if isinstance(matrix, dict) and "cases" in matrix else matrix)
    approved = {c["case_id"]: c for c in entries if c["case_id"] in files["cases"]}
    scope = {
        "runtime": "Ubuntu 24.04 arm64, Gazebo Harmonic 8.15.0, ROS 2 Jazzy, OpenCV 4.6.0",
        "capture": "real native Gazebo RGB8 frames obtained via the ROS camera bridge",
        "replay_delivery": schedule["image_delivery"],
        "motion": "live Gazebo plant via production ROS guard, bridge and DiffDrive",
        "scope_limit": "recorded RGB replay with independent drive plant; simulated timing; no mechanical terminal sensor or physical-device trial",
        "simulation_clock_freshness_s": schedule["freshness_s"],
        "replay_frame_period_sim_s": schedule["period_s"],
        "physics_step_s": .001,
        "configured_realtime_factor": float(re.search(r"<real_time_factor>(.*?)</real_time_factor>",
                                                    (primary/"physics.sdf").read_text())[1]),
        "ros_domain_id": 83, "gz_partition": "arche_janus_video_83",
    }
    source_paths = [
        "janus/ros2/src/arche_experiments/scripts/qr_observer",
        "janus/ros2/src/arche_experiments/scripts/command_guard",
        "janus/ros2/src/arche_experiments/arche_experiments/vision.py",
        "janus/ros2/src/arche_experiments/arche_experiments/commands.py",
        "janus/ros2/src/arche_sim/models/arche_janus/model.sdf",
        "caster-physics-python-v2/qr_docking.py",
    ]
    revision = subprocess.check_output(["git", "-C", source, "rev-parse", "HEAD"], text=True).strip()
    base_revision = subprocess.check_output(["git", "-C", source, "rev-parse", "629bf31"], text=True).strip()
    branch = subprocess.check_output(["git", "-C", source, "branch", "--show-current"], text=True).strip()
    package = {"base_source_revision": base_revision, "revision": revision, "source_branch": branch,
               "source_files_sha256": {p: digest(source/p) for p in source_paths},
               "scope": scope, "case_results": {}, "retained_attempts": {}}
    trace_path = primary / "recording/trace.jsonl"
    live_rows = [json.loads(line) for line in trace_path.read_text().splitlines()]
    head = next(r for r in live_rows if r["kind"] == "origin")
    end = next(r for r in reversed(live_rows) if r["kind"] == "request" and r.get("trigger") == "end")
    package["scope"]["observed_average_realtime_factor"] = ((end["clock_s"]-head["clock_s"]) /
        ((end["wall_ns"]-head["wall_ns"])/1e9))
    report = [
        "# JANUS Q09–Q12 | Gazebo camera replay qualification", "",
        f"The primary run injected {files['recorded_input_frame_count']} RGB8 frames into the production "
        "observer callback. A live ROS guard and a separate Gazebo Harmonic physics server recorded the "
        "command and measured body motion. The original captures used the Gazebo camera and ROS bridge. "
        "Replay reused the same retained frames across trials, so input count is a replay count.", "",
        "| Case | Applicable result | Measured evidence |", "| --- | --- | --- |",
    ]
    for case_id, case in files["cases"].items():
        case_path = analysis/case_id
        clip = load(case_path/"video-index.json")
        criteria = {key: approved.get(case_id, {}).get(key) for key in
                    ("plan_scope_tr", "acceptance_tr", "measured_fields_tr")}
        run_id = f"{case_id}-gazebo-native-video-{primary.name.removeprefix('replay-')}"
        base = {
            "run_id": run_id, "case_id": case_id, "status": case["status"], "scope": scope,
            "criteria": criteria, "base_source_revision": base_revision, "harness_revision": revision,
            "approved_plan_sha256": digest(originals/"approved-plan.md"),
            "qualification_criteria_sha256": digest(originals/"qualification-matrix.json"),
            "termination_reason": end["trigger"],
            "evidence": {p.name: digest(p) for p in sorted(case_path.iterdir()) if p.is_file()
                         and p.name != "manifest.json"},
            "raw_trace": f"../../{primary.name}/recording/trace.jsonl",
            "raw_trace_sha256": files["raw_trace_sha256"],
            "fixture_manifest": "../../fixtures/manifest.json",
            "fixture_manifest_sha256": files["fixture_manifest_sha256"],
            "video_verified_frame_count": clip["verified_frame_count"],
            "terminal_scope": case["q12_positive_terminal_scope"],
        }
        json_write(case_path/"manifest.json", base)
        package["case_results"][case_id] = {
            "case_id": case_id, "run_id": run_id, "status": case["status"],
            "evidence_path": f"{case_id}/manifest.json", "checks": case["checks"],
            "terminal_scope": base["terminal_scope"],
        }
        if case_id == "Q09":
            trial = case["trials"][0]
            mixed = trial["preface"]["valid_frames"]
            duplicates = trial["disturbance"]["outcome_reasons"].get("ambiguous_target", 0)
            description = (f"{mixed} prefacing observations selected the configured target; {duplicates} duplicate "
                           "observations rejected. Both IDs were selected separately on the same native frames in the offline cross-check.")
        elif case_id == "Q10":
            trials = select_trials(case, {"hotspot", "reflection", "exposure_whiteout"})
            count = sum(t["disturbance"]["observed_frames"] for t in trials)
            valid = sum(t["disturbance"]["valid_frames"] for t in trials)
            outputs = span([t["loss_and_stop"]["capture_to_guard_zero_sim_ms"] for t in trials], unit=" sim ms")
            description = f"{valid}/{count} valid observations during strong glare, reflection or whiteout. Guard cut {outputs} from disturbance onset where a run-up was measured."
        elif case_id == "Q11":
            low = select_trials(case, {"low_light"})[0]["disturbance"]
            trials = [t for t in case["trials"] if t["phase_results_classification"] == "required rejection"]
            physical = [t["loss_and_stop"]["body_stop"] for t in trials if t["loss_and_stop"]["body_stop"]]
            p95 = low["capture_to_publish_sim_ms"]["p95"] if low["capture_to_publish_sim_ms"] else None
            description = (f"Low light {low['valid_frames']}/{low['published_frames']} valid, P95 capture-to-publication "
                           f"{span([p95], 1, ' sim ms')}. {len(physical)}/{len(trials)} measured body stops. "
                           f"Travel until settle {span([p['capture_to_settled_path_mm'] for p in physical],1,' mm')}.")
        else:
            count = sum(t["disturbance"]["observed_frames"] for t in case["trials"])
            physical = sum(t["loss_and_stop"]["body_stop"] is not None for t in case["trials"])
            description = (f"{count} observations covering occlusion, digital edge crop and native close-range edge crop. "
                           f"{physical}/3 moving trials measured a settled stop. Contact and charge were unobserved.")
        report.append(f"| [{case_id}]({case_id}/metrics.json) | {status_detail(case)} | {description} |")
    report += ["", "## Scope and retained runs", "",
               "- Direct callback replay used 10 Hz in simulation and a 0.15 Gazebo real-time factor cap. The "
               "0.20 s freshness gate and 1 ms physics step remained active. The trace separately records "
               "Linux monotonic decode and delivery times. Wall-time operating qualification remains open.",
               "- The 20 Hz DDS comparison was terminated after detecting an extra bridge from the native capture "
               "launcher. A clean 10 Hz DDS comparison and a paced 10 Hz DDS comparison record dropped/stale "
               "frames and incomplete stop coverage. All three raw traces are retained.",
               "- The first direct replay hit a wall-time limit near the final recovery and contained a clock-skew "
               "rejection during the strong-glare run-up. A short repeat was terminated to extend the wall-time "
               "budget. Both raw traces are retained.",
               "- The prepatch PNG is retained with its offline outcome. It has no exact capture timestamp attached "
               "to the saved PNG. The first two-station render resolved both placards to ID 01 and is also retained. "
               "The repeat used distinct mesh, material and texture resource names.",
               "- The clipped-exposure fixture's pre-run rejection prediction is retained. The decoder still "
               "recovered the authorized ID at that exposure. Complete whiteout produced detection loss.",
               "- Both rendered texture files were verified against locally signed Ed25519 inputs. The station "
               "QR payloads are plain IDs. Field trust and on-robot authentication remain unverified.",
               "- The qualification gate uses replay images and measured simulator motion. The Q12 plant has no "
               "approved independent dock sensor or contact body. Mechanical docking and charging remain unqualified.",
               "", "## Evidence", ""]
    for cid in files["cases"]:
        report.append(f"- {cid} [frames]({cid}/camera-frames.png) · [outcomes]({cid}/outcomes.png) · "
                      f"[stop trace]({cid}/stop-trace.png) · [lossless delivered pixels]({cid}/delivered-rgb.mkv) · "
                      f"[frame index]({cid}/video-index.json)")
    report.append("- [Q11 measured quality and latency](Q11/quality-latency.png)")
    report.append(f"- [Raw ROS trace](../{primary.name}/recording/trace.jsonl) · "
                  "[fixture strengths and full frame lineage](../fixtures/manifest.json) · "
                  "[offline decoding and station input verification](../offline-scan.json)")
    for name in ("replay-a", "replay-b", "replay-c", "replay-d", "replay-e"):
        file = lane / "results" / name / "recording/trace.jsonl"
        if file.exists() and name != primary.name:
            data = [json.loads(line) for line in file.read_text().splitlines()]
            count = Counter(r["kind"] for r in data)
            package["retained_attempts"][name] = {"path": f"../{name}/recording/trace.jsonl", "sha256": digest(file),
                    "input_count": count["image_sent"], "observation_count": count["observation"],
                    "guard_nonzero_samples": sum(r["kind"] == "guard_output" and r["v_mps"] != 0 for r in data)}
    json_write(analysis / "manifest.json", package)
    (analysis/"RESULTS.md").write_text("\n".join(report) + "\n")


if __name__ == "__main__":
    main()
