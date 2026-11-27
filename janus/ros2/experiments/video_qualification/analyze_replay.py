#!/usr/bin/env python3
"""Join fixture lineage, camera decisions, command enforcement, and measured Gazebo motion."""
import argparse
import bisect
from collections import Counter, defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from qr_camera import Camera, Candidate
from qr_docking import Dock, Observation as DockObservation

TARGET = "JANUS-DOCK-01"
OTHER = "JANUS-DOCK-99"
FRESHNESS = 0.20
SPEED_LIMIT, YAW_RATE_LIMIT, DWELL = 0.02, 0.05, 0.25
NO_TERMINAL = {"mechanical_alignment_source": None, "contact_source": None,
               "charge_source": None, "terminal_mechanism_approved": False}
EXPECTED_LOSS = {"duplicate_ids", "hotspot", "reflection", "exposure_whiteout",
                 "motion_blur", "low_light_motion_blur", "occlusion", "edge_crop",
                 "native_near_edge"}


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")


def write_csv(path, rows):
    columns = list(dict.fromkeys(name for row in rows for name in row))
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def timed(rows, key):
    ordered = sorted(rows, key=lambda row: row[key])
    return ordered, [row[key] for row in ordered]


def window(series, start, stop):
    rows, stamps = series
    return rows[bisect.bisect_left(stamps, start):bisect.bisect_left(stamps, stop)]


def next_row(series, now):
    rows, stamps = series
    i = bisect.bisect_left(stamps, now)
    return rows[i] if i < len(rows) else None


def prior_row(series, now):
    rows, stamps = series
    i = bisect.bisect_right(stamps, now) - 1
    return rows[i] if i >= 0 else None


def cut_delay_ms(first, second):
    return round((second-first)*1000, 3) if first is not None and second is not None else None


def quantiles(values):
    return ({f"p{q:02d}": float(np.percentile(values, q)) for q in (0, 50, 95, 100)}
            if values else None)


def is_zero(row):
    return abs(row["v_mps"]) <= 1e-12 and abs(row["yaw_rate_radps"]) <= 1e-12


def stopped(row):
    return row["planar_speed_mps"] <= SPEED_LIMIT and abs(row["yaw_rate_radps"]) <= YAW_RATE_LIMIT


def interpolate(rows, field, stamp):
    return float(np.interp(stamp, [r["time_s"] for r in rows], [r[field] for r in rows]))


def measured_stop(series, first_zero, capture_s, recovery_s):
    if not first_zero:
        return None
    zero_s = first_zero["clock_s"]
    candidates = window(series, zero_s, recovery_s)
    low_since = last = settled = None
    for row in candidates:
        stamp = row["time_s"]
        if low_since is not None and stamp - last > 0.061:
            low_since = None
        if stopped(row):
            if low_since is None:
                low_since = stamp
            if stamp - low_since >= DWELL - 1e-10:
                settled = stamp
                break
        else:
            low_since = None
        last = stamp
    if settled is None:
        return None
    history = window(series, capture_s - 0.08, settled + 0.08)
    if len(history) < 2 or history[0]["time_s"] > capture_s or history[-1]["time_s"] < settled:
        return None
    distance = {}
    for label, start in (("capture_to_settled", capture_s), ("guard_zero_to_settled", zero_s)):
        crossing = [start, *[row["time_s"] for row in history if start < row["time_s"] < settled],
                    settled]
        xy = [(interpolate(history, "x_m", t), interpolate(history, "y_m", t)) for t in crossing]
        distance[f"{label}_path_mm"] = 1000 * sum(math.dist(a, b) for a, b in zip(xy, xy[1:]))
        distance[f"{label}_displacement_mm"] = 1000 * math.dist(xy[0], xy[-1])
    return {"below_stop_speed_since_sim_s": low_since, "settled_sim_s": settled,
            "first_zero_to_below_stop_speed_sim_ms": cut_delay_ms(zero_s, low_since),
            "first_zero_to_settled_sim_ms": cut_delay_ms(zero_s, settled), **distance}


def shadow_dock(frames, telemetry):
    camera = Camera(fx_px=554.2562866, fy_px=554.2563057, forward_m=.25, height_m=.31)
    dock = Dock(camera, .24*21/29)
    rows = []
    observed = [frame for frame in frames if frame["observed"]]
    if not observed:
        return rows
    origin = observed[0]["observation_receipt_sim_s"]
    for frame in observed:
        data = frame["observation"]
        now = frame["observation_receipt_sim_s"]
        pos = next_row(telemetry, now)
        candidate = (Candidate(data["id"], tuple(map(tuple, data["corners_px"])), True, 1.0)
                     if data["valid"] else None)
        obs = DockObservation("valid" if data["valid"] else data["reason"],
                              candidate=candidate, error=data["normalized_error"])
        v, yaw, reason = dock.update(now-origin, obs, bool(pos and stopped(pos)), (None, None))
        rows.append({"frame_sequence": frame["frame_sequence"], "clock_s": now,
                     "shadow_phase": dock.phase, "shadow_reason": reason,
                     "shadow_requested_v_mps": v, "shadow_requested_yaw_radps": yaw,
                     "aligned": dock.aligned, "contact": dock.contact,
                     "charge_confirmed": dock.charge_confirmed, "success": dock.phase == "success",
                     **NO_TERMINAL})
    return rows


def frame_rows(by_kind, manifest, guard):
    observed = {o["frame_sequence"]: o for o in by_kind["observation"] if o["frame_sequence"] is not None}
    enforced = {r["frame_sequence"]: r for r in by_kind["request"] if r["frame_sequence"] is not None}
    outputs = []
    for src in by_kind["image_sent"]:
        key = src["frame_sequence"]
        spec = manifest["fixtures"][src["variant"]]["frames"][src["fixture_frame_index"]]
        obs = observed.get(key)
        request = enforced.get(key)
        next_guard = next_row(guard, request["wall_ns"]) if request else None
        if obs and round(obs["capture_time_s"]*1e9) != round(src["capture_time_s"]*1e9):
            raise ValueError("Observation joined to different capture clock")
        image = {k: v for k, v in src.items() if k not in ("kind",)}
        image.update({"observed": obs is not None, "observation": obs,
            "candidate_count": len(obs["candidates"]) if obs else None,
            "detected": bool(obs and obs["candidates"]),
            "selected_id": obs["id"] if obs else None, "valid": bool(obs and obs["valid"]),
            "reason": obs["reason"] if obs else "no_observer_output",
            "decode_ms": obs["decode_ms"] if obs else None,
            "capture_to_arrival_sim_ms": cut_delay_ms(src["capture_time_s"], obs["arrival_time_s"]) if obs else None,
            "capture_to_publish_sim_ms": cut_delay_ms(src["capture_time_s"], obs["publish_time_s"]) if obs else None,
            "capture_to_control_sim_ms": cut_delay_ms(src["capture_time_s"], request["now_s"]) if request else None,
            "publisher_to_control_wall_ms": (request["wall_ns"]-src["wall_ns"])/1e6 if request else None,
            "observation_receipt_sim_s": max(obs["publish_time_s"], obs["clock_s"]) if obs else None,
            "requested_v_mps": request["v_mps"] if request else None,
            "request_reason": request["reason"] if request else None,
            "next_guard_receipt_sim_s": next_guard["clock_s"] if next_guard else None,
            "control_to_next_guard_wall_ms": (next_guard["wall_ns"]-request["wall_ns"])/1e6 if next_guard else None,
            "next_guard_v_mps": next_guard["v_mps"] if next_guard else None,
            "next_guard_yaw_rate_radps": next_guard["yaw_rate_radps"] if next_guard else None,
            "source_png_sha256": spec["source_png_sha256"],
            "geometric_visible_corner_count": spec.get("geometric_visible_corner_count"),
            "visible_corner_annotation_origin": spec.get("annotation_origin"),
            **spec["image_quality"]})
        outputs.append(image)
    return outputs


def stage_metrics(frames):
    seen = [r for r in frames if r["observed"]]
    ages_ms = [r["capture_to_publish_sim_ms"] for r in seen]
    valid = [r for r in seen if r["valid"]]
    decoded = [r for r in seen if any(c["id"] for c in r["observation"]["candidates"])]
    return {"published_frames": len(frames), "observed_frames": len(seen),
            "unobserved_frames": len(frames)-len(seen), "detected_frames": sum(r["detected"] for r in seen),
            "decoded_id_frames": len(decoded), "valid_frames": len(valid),
            "valid_fraction_of_published": len(valid)/len(frames) if frames else None,
            "detected_fraction_of_observed": sum(r["detected"] for r in seen)/len(seen) if seen else None,
            "selected_ids": dict(Counter(r["selected_id"] for r in seen if r["selected_id"])),
            "outcome_reasons": dict(Counter(r["reason"] for r in frames)),
            "wrong_id_accepted_frames": sum(r["selected_id"] != TARGET for r in valid),
            "valid_frames_over_200_ms": sum(r["capture_to_publish_sim_ms"] > 200.001 for r in valid),
            "capture_to_publish_sim_ms": quantiles(ages_ms),
            "capture_to_control_sim_ms": quantiles([r["capture_to_control_sim_ms"] for r in seen
                                                    if r["capture_to_control_sim_ms"] is not None]),
            "publisher_to_control_wall_ms": quantiles([r["publisher_to_control_wall_ms"] for r in seen
                                                        if r["publisher_to_control_wall_ms"] is not None]),
            "decode_wall_ms": quantiles([r["decode_ms"] for r in seen if r["decode_ms"] is not None]),
            "accepted_area_ratio": quantiles([r["observation"]["quality_area_ratio"] for r in valid]),
            "roi_luma_span": quantiles([r["roi_luma_span_p95_p05"] for r in frames]),
            "roi_clipped_white_fraction": quantiles([r["roi_clipped_white_fraction"] for r in frames]),
            "roi_laplacian_variance": quantiles([r["roi_laplacian_variance"] for r in frames]),
            "geometric_visible_corner_counts": sorted({r["geometric_visible_corner_count"]
                                                      for r in frames}) if frames else [],
            "invalid_capture_span_s": (seen[-1]["capture_time_s"]-seen[0]["capture_time_s"]
                                       if seen and not valid else None)}


def trial_metrics(trial, frames, requests, guard_time, guard_wall, telemetry, origin, shadow):
    perturb, recover = (origin + trial[key] for key in ("disturbance_s", "recovery_s"))
    observed = [row for row in frames if row["phase"] == "disturbance" and row["observed"]]
    phase = {key: stage_metrics([r for r in frames if r["phase"] == key])
             for key in ("preface", "disturbance", "recovery")}
    lead_guards = window(guard_time, perturb-.45, perturb)
    lead_motion = window(telemetry, perturb-.45, perturb)
    lead_max_v = max((r["v_mps"] for r in lead_guards), default=0)
    lead_max_body_v = max((r["planar_speed_mps"] for r in lead_motion), default=0)
    at_onset_body = prior_row(telemetry, perturb)
    if at_onset_body is not None and perturb - at_onset_body["time_s"] > .061:
        at_onset_body = None
    at_onset_guard = prior_row(guard_time, perturb)
    onset_speed = at_onset_body["planar_speed_mps"] if at_onset_body else None
    onset_guard_v = at_onset_guard["v_mps"] if at_onset_guard else None
    in_motion_at_onset = bool(onset_speed is not None and onset_speed > SPEED_LIMIT
                              and onset_guard_v is not None and onset_guard_v > 0)
    first_invalid = next((r for r in observed if not r["valid"]), None)
    prior_stop = next((r for r in requests if r["reason"] == "expired_at_control"
                       and r["now_s"] >= perturb and r["now_s"] < recover and is_zero(r)), None)
    first_request = next((r for r in requests if first_invalid is not None and
                          r["frame_sequence"] == first_invalid["frame_sequence"] and is_zero(r)), None)
    first_cut = prior_stop if (prior_stop is not None and
                              (first_request is None or prior_stop["wall_ns"] < first_request["wall_ns"])) else first_request
    if first_cut:
        zero_candidates = [r for r in window(guard_wall, first_cut["wall_ns"],
                          first_cut["wall_ns"] + 3_000_000_000) if is_zero(r)]
        first_zero = next(iter(zero_candidates), None)
    else:
        first_zero = None
    stay_zero = (first_zero is not None and
                 all(is_zero(r) for r in window(guard_time, first_zero["clock_s"], recover)))
    physical = measured_stop(telemetry, first_zero, perturb, recover) if in_motion_at_onset else None
    mechanical_checks = {key: any(r[key] for r in shadow)
                         for key in ("aligned", "contact", "charge_confirmed", "success")}
    bad_forward_ids = [r["selected_id"] for r in requests if r["v_mps"] > 0 and r["selected_id"] != TARGET]
    invalid_sequences = {f["frame_sequence"] for f in frames if f["observed"] and not f["valid"]}
    invalid_requests = sum(r["frame_sequence"] in invalid_sequences and not is_zero(r) for r in requests)
    prephase = [r for r in frames if r["phase"] == "preface" and r["valid"]]
    candidate_check = True
    if trial["case"] == "Q09":
        good = [r for r in prephase if sorted(c["id"] or "" for c in r["observation"]["candidates"]) == [TARGET, OTHER]]
        duplicate = [r for r in observed if r["reason"] == "ambiguous_target" and
                     [c["id"] for c in r["observation"]["candidates"]] == [TARGET, TARGET]]
        candidate_check = bool(good and duplicate) and len(good) == len(prephase)
    hard_loss = trial["stimulus"] in EXPECTED_LOSS or trial["injected_delay_s"] > 0
    invalid_phase_bounds = None
    if hard_loss and first_zero and observed:
        invalid_phase_bounds = {"first_invalid_capture_sim_s": first_invalid["capture_time_s"] if first_invalid else None,
                                "last_observed_capture_sim_s": observed[-1]["capture_time_s"],
                                "guard_zero_until_recovery_sim_s": recover-first_zero["clock_s"]}
    return {**trial, **phase,
            "phase_results_classification": "required rejection" if hard_loss else "quality characterization",
            "candidate_contract_pass": candidate_check,
            "prior_target_motion": {"peak_guarded_v_mps": lead_max_v, "peak_body_speed_mps": lead_max_body_v,
                                    "guard_v_at_onset_mps": onset_guard_v,
                                    "body_speed_at_onset_mps": onset_speed, "in_motion_at_onset": in_motion_at_onset},
            "safety": {"positive_requests_for_wrong_identity": len(bad_forward_ids),
                       "invalid_frame_nonzero_request_count": invalid_requests,
                       "shadow_dock_false_events": mechanical_checks,
                       "freshness_violations": sum(s["valid_frames_over_200_ms"] for s in phase.values())},
            "loss_and_stop": {"first_invalid_observation": first_invalid["reason"] if first_invalid else None,
                "first_invalid_observation_clock_s": first_invalid["observation_receipt_sim_s"] if first_invalid else None,
                "first_zero_request_reason": first_cut["reason"] if first_cut else None,
                "first_zero_request_sim_s": first_cut["now_s"] if first_cut else None,
                "first_zero_guard_sim_s": first_zero["clock_s"] if first_zero else None,
                "capture_to_zero_request_sim_ms": cut_delay_ms(perturb, first_cut["now_s"]) if first_cut else None,
                "capture_to_guard_zero_sim_ms": cut_delay_ms(perturb, first_zero["clock_s"]) if first_zero else None,
                "request_to_guard_zero_wall_ms": (first_zero["wall_ns"]-first_cut["wall_ns"])/1e6 if first_zero else None,
                "invalid_observation_same_callback_zero": bool(first_request),
                "guard_remained_zero_through_disturbance": stay_zero,
                "invalid_phase_bounds": invalid_phase_bounds, "body_stop": physical},
            **NO_TERMINAL}


def evaluate(case, trials, offline):
    safe = bool(trials) and all(
        not any(t["safety"]["shadow_dock_false_events"].values()) and
        not t["safety"]["positive_requests_for_wrong_identity"] and
        not t["safety"]["invalid_frame_nonzero_request_count"] and
        not t["safety"]["freshness_violations"] and
        all(not t[phase]["wrong_id_accepted_frames"] for phase in ("preface", "disturbance", "recovery"))
        for t in trials)
    rejected = [t for t in trials if t["phase_results_classification"] == "required rejection"]
    stopped_after_loss = bool(rejected) and all(
        t["disturbance"]["observed_frames"] and not t["disturbance"]["valid_frames"] and
        t["loss_and_stop"]["invalid_observation_same_callback_zero"] and
        t["loss_and_stop"]["guard_remained_zero_through_disturbance"] and
        t["loss_and_stop"]["body_stop"] is not None for t in rejected)
    checks = {"no_false_acceptance_or_invalid_forward": safe}
    if case == "Q09":
        checks.update({"native_mixed_and_duplicate_identity": bool(trials) and all(t["candidate_contract_pass"] for t in trials),
                       "alternate_target_selected_offline": bool(offline["alternate_target"]) and all(
                           r["valid"] and r["id"] == OTHER for r in offline["alternate_target"]),
                       "duplicate_enforced_zero_forward": bool(rejected) and all(
                           t["loss_and_stop"]["invalid_observation_same_callback_zero"] and
                           t["loss_and_stop"]["guard_remained_zero_through_disturbance"] for t in rejected)})
    elif case == "Q10":
        checks.update({"loss_enforced_for_hotspot_reflection_and_whiteout": stopped_after_loss and len(rejected) == 3,
                       "all_stimulus_outputs_measured": bool(trials) and all(t["disturbance"]["observed_frames"] for t in trials)})
    elif case == "Q11":
        low = [t for t in trials if t["stimulus"] == "low_light" and not t["injected_delay_s"]]
        stale = [t for t in trials if t["injected_delay_s"] > 0]
        checks.update({"low_light_quality_and_latency_measured": bool(low) and all(
                           t["disturbance"]["valid_frames"] and
                           t["disturbance"]["capture_to_publish_sim_ms"] is not None for t in low),
                       "motion_stopped_and_settled_repeatedly": stopped_after_loss and len(rejected) == 5,
                       "late_delivery_rejected": bool(stale) and all(
                           t["disturbance"]["outcome_reasons"].get("stale_frame", 0) > 0 for t in stale)})
    else:
        checks.update({"occlusion_and_edge_loss_stopped_and_settled": stopped_after_loss and len(rejected) == 3,
                       "two_remaining_corners_verified": bool(rejected) and all(
                           t["disturbance"]["geometric_visible_corner_counts"] == [2] for t in rejected)})
    qualification = "passed" if all(checks.values()) else "partial_or_failed"
    return {"case": case, "status": qualification, "checks": checks,
            "q12_positive_terminal_scope": ("unqualified: no approved independent terminal sensing/contact input"
                                            if case == "Q12" else None),
            "trials": trials}


def read_trace(recording):
    by_kind = defaultdict(list)
    for line in recording.read_text().splitlines():
        row = json.loads(line)
        by_kind[row["kind"]].append(row)
    if len(by_kind["origin"]) != 1:
        raise ValueError("Trace does not identify an unambiguous simulation origin")
    return by_kind, by_kind["origin"][0]["simulation_origin_s"]


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("replay", type=Path)
    cli.add_argument("fixtures", type=Path)
    cli.add_argument("offline_scan", type=Path)
    cli.add_argument("out", type=Path)
    args = cli.parse_args()
    trace = args.replay / "trace.jsonl"
    by_kind, origin = read_trace(trace)
    fixture_manifest = json.loads(args.fixtures.read_text())
    offline = json.loads(args.offline_scan.read_text())
    schedule = json.loads((args.replay / "schedule.json").read_text())
    guard_time = timed(by_kind["guard_output"], "clock_s")
    guard_wall = timed(by_kind["guard_output"], "wall_ns")
    telemetry = timed(by_kind["ground_truth"], "time_s")
    rows = frame_rows(by_kind, fixture_manifest, guard_wall)
    summary = {"simulation_origin_s": origin, "raw_trace_sha256": digest(trace),
               "fixture_manifest_sha256": digest(args.fixtures), "rows_by_kind": {k: len(v) for k,v in by_kind.items()},
               "replay_period_sim_s": schedule["period_s"], "expected_input_frame_count": schedule["expected_frames"],
               "recorded_input_frame_count": len(rows), "physical_stop_criteria": {
                   "planar_speed_max_mps": SPEED_LIMIT, "yaw_rate_max_radps": YAW_RATE_LIMIT, "dwell_s": DWELL,
                   "max_ground_truth_sample_gap_s": .061},
               "cases": {}}
    args.out.mkdir(parents=True, exist_ok=True)
    for case in ("Q09", "Q10", "Q11", "Q12"):
        case_dir = args.out / case
        case_dir.mkdir(exist_ok=True)
        case_rows = [r for r in rows if r["case"] == case]
        if not case_rows:
            continue
        case_trials = [t for t in schedule["suite"] if t["case"] == case]
        start, end = origin+case_trials[0]["start_s"], origin+case_trials[-1]["end_s"]
        request_rows = [r for r in by_kind["request"] if start <= r["clock_s"] < end]
        trace_ground = window(telemetry, start, end)
        timeline = window(guard_time, start, end)
        metrics, dock_shadow = [], []
        for trial in case_trials:
            frame_list = [r for r in case_rows if r["trial"] == trial["trial"]]
            requests = [r for r in request_rows if r["trial"] == trial["trial"]]
            shadow_rows = shadow_dock(frame_list, telemetry)
            metrics.append(trial_metrics(trial, frame_list, requests, guard_time, guard_wall,
                                         telemetry, origin, shadow_rows))
            dock_shadow.extend(shadow_rows)
        summary["cases"][case] = evaluate(case, metrics, offline)
        dedup = {r["clock_s"]: r for r in timeline}
        command_origin = min(dedup) if dedup else start
        state_origin = trace_ground[0]["time_s"] if trace_ground else start
        summary["cases"][case]["csv_time_origins_in_sim_s"] = {
            "commands.csv": command_origin, "results.csv": state_origin,
        }
        write_json(case_dir / "metrics.json", summary["cases"][case])
        with (case_dir/"frames.jsonl").open("w") as sink:
            for row in case_rows:
                sink.write(json.dumps(row, allow_nan=False)+"\n")
        write_csv(case_dir/"frames.csv", [{**{k:v for k,v in row.items() if k not in ("observation",)},
                  "candidate_ids": json.dumps([c["id"] for c in (row["observation"] or {}).get("candidates", [])]),
                  "candidate_corners_px": json.dumps([c["corners_px"] for c in (row["observation"] or {}).get("candidates", [])])}
                  for row in case_rows])
        write_csv(case_dir/"commands.csv", [{"time_s": stamp-command_origin, "v_mps": r["v_mps"],
                  "yaw_rate_radps": r["yaw_rate_radps"]} for stamp,r in dedup.items()])
        write_csv(case_dir/"command_requests.csv", request_rows)
        write_csv(case_dir/"results.csv", [{**r, "sim_header_time_s": r["time_s"],
                                            "time_s": r["time_s"]-state_origin} for r in trace_ground])
        write_csv(case_dir/"terminal_shadow.csv", dock_shadow)
    write_json(args.out / "qualification.json", summary)


if __name__ == "__main__":
    main()
