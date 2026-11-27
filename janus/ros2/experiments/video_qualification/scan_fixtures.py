#!/usr/bin/env python3
"""Characterize the pixels separately from live ROS delivery and verify station assets."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
from time import perf_counter_ns

import cv2

from arche_experiments.vision import Decoder, Observation, TARGET_ID

ALT_ID = "JANUS-DOCK-99"


def scan(path, target, u, decoder):
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise OSError(path)
    height, width = image.shape[:2]
    start = perf_counter_ns()
    obs = decoder.decode(image, Observation(0.0, 0.0, width, height, target, u))
    return {"decode_ms": (perf_counter_ns() - start) / 1e6, **obs.as_row()}


def verify_assets(root, openssl):
    manifest = json.loads((root / "manifest.json").read_text())
    decoder = Decoder()
    result = []
    for station in manifest["stations"]:
        png = root / station["png"]
        rendered = root.parent / station["rendered_asset"]
        digest = hashlib.sha256(rendered.read_bytes()).hexdigest()
        signature = subprocess.run([
            openssl, "pkeyutl", "-verify", "-pubin", "-rawin",
            "-inkey", root / "qualification-public.pem", "-in", rendered,
            "-sigfile", root / station["signature"],
        ], capture_output=True, text=True)
        qr = scan(png, station["id"], 232.0, decoder)
        checks = {"asset_hash_matches": digest == station["sha256"],
                  "retained_png_byte_identical": png.read_bytes() == rendered.read_bytes(),
                  "signature_verified": signature.returncode == 0,
                  "id_verified": qr["valid"] and qr["id"] == station["id"]}
        result.append({**station, **checks, "fixture_input_decode": qr})
        if not all(checks.values()):
            raise ValueError(f"Asset verification failed: {station['id']}")
    return {"algorithm": manifest["algorithm"], "trust": manifest["trust"], "checks": result}


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("fixture_dir", type=Path)
    cli.add_argument("out", type=Path)
    cli.add_argument("--originals", type=Path)
    cli.add_argument("--openssl", default="openssl")
    args = cli.parse_args()
    cv2.setNumThreads(1)
    decoder = Decoder()
    manifest = json.loads((args.fixture_dir / "manifest.json").read_text())
    results = {"opencv": cv2.__version__, "execution_scope": "offline pixels; no ROS or motion", "scans": {}}
    for name, fixture in manifest["fixtures"].items():
        rows = []
        for spec in fixture["frames"]:
            png = args.fixture_dir / spec["path"]
            row = scan(png, TARGET_ID, 320.0, decoder)
            rows.append({"path": spec["path"], "frame_index": spec["frame_index"], **row})
        results["scans"][name] = rows
    results["alternate_target"] = [
        {"path": row["path"],
         **scan(args.fixture_dir / row["path"], ALT_ID, 515.0, decoder)}
        for row in manifest["fixtures"]["two_ids"]["frames"]
    ]
    if args.originals:
        results["station_input_verification"] = verify_assets(
            args.originals / "native/scene/station-inputs", args.openssl)
        bad = args.originals / "native-attempt-01/capture/mixed_ids"
        results["first_renderer_attempt_mixed_scene"] = [
            {"path": str(p.relative_to(args.originals)), **scan(p, TARGET_ID, 320.0, decoder)}
            for p in sorted(bad.glob("*.png"))
        ]
    args.out.write_text(json.dumps(results, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
