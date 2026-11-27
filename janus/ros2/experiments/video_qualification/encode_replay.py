#!/usr/bin/env python3
"""Encode delivered PNG sequences and verify lossless frame digests after decoding."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile


def encode(frames, fixture_root, out, ffmpeg, rate):
    with tempfile.TemporaryDirectory(dir=out) as scratch:
        folder = Path(scratch)
        for index, row in enumerate(frames):
            source = (fixture_root / row["fixture_path"]).resolve()
            (folder / f"{index:06d}.png").symlink_to(source)
        video = out / "delivered-rgb.mkv"
        subprocess.run([ffmpeg, "-hide_banner", "-loglevel", "error", "-y", "-framerate", str(rate),
                        "-i", folder / "%06d.png", "-an", "-threads", "1",
                        "-c:v", "ffv1", "-level", "3", "-pix_fmt", "bgr0", video], check=True)
    child = subprocess.Popen([ffmpeg, "-hide_banner", "-loglevel", "error", "-i", video,
                              "-f", "rawvideo", "-pix_fmt", "rgb24", "-threads", "1", "-"],
                             stdout=subprocess.PIPE)
    timeline = []
    size = 640 * 480 * 3
    try:
        for i, row in enumerate(frames):
            raw = child.stdout.read(size)
            digest = hashlib.sha256(raw).hexdigest()
            if len(raw) != size or digest != row["rgb8_sha256"]:
                raise ValueError(f"Lossless verification failed on frame {i}")
            timeline.append({"video_frame": i, "video_time_s": i / rate,
                              **{k: row[k] for k in ("frame_sequence", "phase", "variant",
                                 "capture_time_s", "fixture_path", "rgb8_sha256", "reason")}})
        if child.stdout.read(1):
            raise ValueError("Video has more frames than the trace")
    finally:
        child.stdout.close()
        if child.wait() != 0:
            raise RuntimeError("FFmpeg lossless verification process failed")
    manifest = {"file": video.name, "codec": "FFV1 lossless", "rate_fps": rate,
                "clock": "Capture order by published RGB input. Intermission and injection delays are in the trace.",
                "verified_frame_count": len(timeline), "sha256": hashlib.sha256(video.read_bytes()).hexdigest(),
                "frames": timeline}
    (out / "video-index.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"{out.name}: verified {len(timeline)} lossless RGB frames")


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("analysis", type=Path)
    cli.add_argument("fixture_root", type=Path)
    cli.add_argument("--ffmpeg", default=shutil.which("ffmpeg"))
    args = cli.parse_args()
    summary = json.loads((args.analysis / "qualification.json").read_text())
    rate = int(round(1.0 / summary["replay_period_sim_s"]))
    for case in summary["cases"]:
        out = args.analysis / case
        rows = [json.loads(line) for line in (out/"frames.jsonl").read_text().splitlines()]
        encode(rows, args.fixture_root, out, args.ffmpeg, rate)


if __name__ == "__main__":
    main()
