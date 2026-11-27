#!/usr/bin/env python3
"""Check provenance, per-frame joins, command authority, and evidence payloads."""
import argparse
import hashlib
import json
from pathlib import Path
import re

from PIL import Image


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def expect(condition, description):
    if not condition:
        raise ValueError(description)


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def load(path):
    return json.loads(path.read_text())


def file_stats(path):
    with Image.open(path) as image:
        rgb = image.convert("RGB")
        return rgb.size, hashlib.sha256(rgb.tobytes()).hexdigest()


def verify_graph(graph):
    text = (graph / "cmd-vel-graph.txt").read_text()
    pubs = text.split("Subscription count:", 1)[0]
    expect(re.search(r"Publisher count: 1\b", pubs) and "janus_command_guard" in pubs,
           "Expected single production guard publisher on /cmd_vel")
    expect(text.count("Node name: ros_gz_bridge") == 1, "Expected exactly one Gazebo velocity bridge")


def verify_runtime(base, raw):
    source = base / "checkout/janus/ros2"
    files = {
        "source-nodes/qr_observer": source / "src/arche_experiments/scripts/qr_observer",
        "source-nodes/command_guard": source / "src/arche_experiments/scripts/command_guard",
        "fixtures/manifest.json": base / "results/fixtures/manifest.json",
        "scripts/live_replay.py": source / "experiments/video_qualification/live_replay.py",
        "scripts/run_live_replay.sh": source / "experiments/video_qualification/run_live_replay.sh",
    }
    for name in ("vision", "commands", "ros_fields"):
        files[f"source/arche_experiments/{name}.py"] = source / f"src/arche_experiments/arche_experiments/{name}.py"
    recorded = {name: checksum for checksum, name in
                (line.split(maxsplit=1) for line in (raw/"runtime-inputs.sha256").read_text().splitlines())}
    for name, path in files.items():
        expect(recorded[name] == sha(path), f"Guest input differs from pinned lane source: {name}")
    captured_scene = base / "originals/native/scene/models/arche_janus/model.sdf"
    expect(any(name.endswith("/arche_janus/model.sdf") and checksum == sha(captured_scene)
               for name, checksum in recorded.items()), "Drive plant differs from original Gazebo source model")


def verify_forward_requests(trace, sent):
    observations = {}
    count = 0
    for row in trace:
        if row["kind"] == "observation" and row["frame_sequence"] in sent:
            observations[sent[row["frame_sequence"]]["trial"]] = row
        if row["kind"] == "request" and row["v_mps"] > 0:
            count += 1
            last = observations.get(row["trial"])
            expect(last and last["valid"] and last["id"] == "JANUS-DOCK-01",
                   "Positive request without a valid authorized station observation")
            age = row["now_s"] - last["capture_time_s"]
            expect(0 <= age <= .200001, "Positive request used an expired observation")
    return count


def verify(base, raw, analysis):
    original = base / "originals"
    fixture_root = base / "results/fixtures"
    raw = base / raw
    analysis = base / analysis
    digest_manifest = load(fixture_root / "manifest.json")
    scan = load(base / "results/offline-scan.json")
    qualification = load(analysis / "qualification.json")
    expect(qualification["fixture_manifest_sha256"] == sha(fixture_root / "manifest.json"), "Fixture manifest changed")
    expect(qualification["raw_trace_sha256"] == sha(raw / "recording/trace.jsonl"), "Raw trace changed")
    verify_graph(raw)
    verify_runtime(base, raw)

    assets = scan["station_input_verification"]["checks"]
    expect({a["id"] for a in assets} == {"JANUS-DOCK-01", "JANUS-DOCK-99"}, "Both source QR images required")
    for station in assets:
        expect(all(station[key] for key in
                   ("asset_hash_matches", "retained_png_byte_identical", "signature_verified", "id_verified")),
               f"Station verification missing: {station['id']}")

    untouched = 0
    fixture_map = {}
    for fixture in digest_manifest["fixtures"].values():
        for spec in fixture["frames"]:
            path = fixture_root / spec["path"]
            original_png = original / spec["original_path"]
            expect(sha(path) == spec["png_sha256"], f"Changed replay input: {path}")
            expect(sha(original_png) == spec["source_png_sha256"], f"Changed native input: {original_png}")
            size, rgb = file_stats(path)
            expect(size == (640, 480), f"Wrong native dimensions: {path}")
            fixture_map[spec["path"]] = rgb
            capture = spec["native_capture"]
            if capture:
                _, source_rgb = file_stats(original_png)
                expect(capture["rgb_sha256"] == source_rgb, f"Native ROS RGB differs from saved PNG: {original_png}")
            if fixture["transform"]["kind"] == "unmodified native RGB":
                expect(path.read_bytes() == original_png.read_bytes(), f"Unmodified fixture was changed: {path}")
                untouched += 1

    actual = rows(raw / "recording/trace.jsonl")
    sent = {r["frame_sequence"]: r for r in actual if r["kind"] == "image_sent"}
    positive_samples = verify_forward_requests(actual, sent)
    joined = {}
    for case in ("Q09", "Q10", "Q11", "Q12"):
        case_root = analysis / case
        frames = rows(case_root / "frames.jsonl")
        case_video = load(case_root / "video-index.json")
        expect(case_video["verified_frame_count"] == len(frames), "Video frame count does not match trace")
        expect(case_video["sha256"] == sha(case_root / case_video["file"]), "RGB video bytes changed")
        for frame, video_frame in zip(frames, case_video["frames"], strict=True):
            sequence = frame["frame_sequence"]
            expect(sequence not in joined, "Repeated frame sequence")
            joined[sequence] = frame
            expect(frame["rgb8_sha256"] == fixture_map[frame["fixture_path"]], "ROS replay bytes changed")
            expect(frame["fixture_path"] == sent[sequence]["fixture_path"], "Replay path does not match")
            for key in ("capture_time_s", "rgb8_sha256", "frame_sequence"):
                expect(frame[key] == video_frame[key], f"FFV1 video index mismatch: {key}")
            if frame["valid"]:
                expect(frame["selected_id"] == "JANUS-DOCK-01", "Wrong target accepted")
                expect(frame["capture_to_publish_sim_ms"] <= 200.001, "Stale frame accepted")
            if frame["observed"] and not frame["valid"]:
                expect(frame["requested_v_mps"] in (None, 0.0), "Invalid observation requested forward motion")
        for filename in ("camera-frames.png", "outcomes.png", "stop-trace.png"):
            with Image.open(case_root / filename) as graph:
                graph.verify()
    expect(len(joined) == len(sent) == qualification["expected_input_frame_count"], "Replay incomplete")
    expect({r.get("image_delivery") for r in sent.values()} == {load(raw / "recording/schedule.json")["image_delivery"]},
           "Mixed or unclear image delivery tier")
    expect((original / "native-attempt-01/capture/mixed_ids/000.png").is_file(), "Original failed render was discarded")
    for name in ("replay-a", "replay-b", "replay-c"):
        expect((base/"results"/name/"recording/trace.jsonl").stat().st_size > 0, f"Comparison attempt {name} missing")
    print(f"Verified {len(fixture_map)} frame files, {untouched} unmodified native PNGs and "
          f"{len(joined)} lossless live replay inputs. /cmd_vel authority is unique.")
    print(f"Independently checked the authorized ID and freshness for {positive_samples} positive requests.")
    print("Case checks", {case: result["status"] for case, result in qualification["cases"].items()})


if __name__ == "__main__":
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("lane", type=Path)
    cli.add_argument("raw", type=Path)
    cli.add_argument("analysis", type=Path)
    args = cli.parse_args()
    verify(args.lane, args.raw, args.analysis)
