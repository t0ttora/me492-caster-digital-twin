"""Independently evaluate all four edges and four turns from a native raw trace."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def verify(path, direction):
    names = ("time_s", "axle_x_m_1", "axle_y_m_1", "axle_angles_deg_1", "control_16")
    data = np.genfromtxt(path, delimiter=",", names=True, usecols=names)
    time = data["time_s"]
    xy = np.column_stack((data["axle_x_m_1"], data["axle_y_m_1"]))
    yaw = np.unwrap(np.deg2rad(data["axle_angles_deg_1"]))
    dx = np.diff(xy, axis=0)
    dt = np.diff(time)
    speed = np.r_[0, np.linalg.norm(dx, axis=1) / dt]
    yaw_rate = np.r_[0, np.diff(yaw) / dt]
    if not np.allclose(dt, .001, rtol=0, atol=1e-8) or not np.isfinite(xy).all():
        raise ValueError("A finite contiguous 1 ms native trace is required")

    targets = np.array(((.5, 0), (.5, 0), (.5, .5), (.5, .5), (0, .5), (0, .5), (0, 0), (0, 0)))
    targets[:, 1] *= direction
    target_yaw = direction * np.repeat(np.arange(5) * np.pi / 2, 2)[1:-1]
    latch = data["control_16"]
    results = []
    hold = None
    for index, (position, heading) in enumerate(zip(targets, target_yaw), start=1):
        hits = np.flatnonzero(latch >= index)
        if not len(hits):
            results.append({"gate": index, "executed": False, "passed": False})
            continue

        end = hits[0]
        effective = (time >= time[end] - .25 - 1e-10) & (time <= time[end])
        pos = np.linalg.norm(xy - position, axis=1)
        wrapped = (yaw - heading + np.pi) % (2 * np.pi) - np.pi
        pos_gate, yaw_gate = (.02, .05) if index == 8 else (.05, np.deg2rad(5))
        limits = (pos_gate, yaw_gate, .02, .05)
        observations = np.column_stack((pos, np.abs(wrapped), speed, np.abs(yaw_rate)))
        peak = observations[effective].max(axis=0)
        results.append({
            "gate": index, "kind": "edge" if index % 2 else "turn",
            "time_s": float(time[end]), "window_rows": int(np.count_nonzero(effective)),
            "target_xy_m": position.tolist(), "target_yaw_rad": float(heading),
            "peak_abs": dict(zip(("position_m", "yaw_rad", "planar_speed_mps", "yaw_rate_radps"), peak.tolist())),
            "executed": True, "passed": bool(np.all(peak <= limits)),
        })
        if index == 8:
            hold = {"final_in_bounds": bool(np.all(observations[-1] <= limits)),
                    "remained_in_bounds_after_gate": bool(np.all(observations[end:] <= limits)),
                    "final_position_m": float(pos[-1]), "final_yaw_rad": float(wrapped[-1]),
                    "final_planar_speed_mps": float(speed[-1]),
                    "final_yaw_rate_radps": float(yaw_rate[-1])}

    return {
        "raw_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "direction": "ccw" if direction > 0 else "cw",
        "observed_gate_count": int(np.max(latch)),
        "independent_gate_passes": sum(gate["passed"] for gate in results),
        "criteria": "0.25 s raw dwell, edge/turn pose ≤5 cm/5°, terminal ≤2 cm/0.05 rad, planar speed ≤0.02 m/s and yaw rate ≤0.05 rad/s",
        "gates": results,
        "terminal_hold": hold,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("native_csv", type=Path)
    parser.add_argument("direction", choices=("cw", "ccw"))
    parser.add_argument("output_json", type=Path)
    args = parser.parse_args()
    result = verify(args.native_csv, -1 if args.direction == "cw" else 1)
    args.output_json.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(result["direction"], result["observed_gate_count"], result["independent_gate_passes"],
          result["terminal_hold"])
