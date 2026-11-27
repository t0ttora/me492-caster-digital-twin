"""Compute run-level acceptance and record measurable scope gaps."""
import math

import numpy as np

from qr_docking import Dock
from qr_camera import DOCK_ID
from scenarios import STOP_INTERLOCKS, progress, segment_duration
from twin import advance_pose, wrap


def wall_clearances(xy, yaw, bounds, half_extents=(.25, .205)):
    xmin, xmax, ymin, ymax = bounds
    hx, hy = half_extents
    ex = hx*abs(np.cos(yaw))+hy*abs(np.sin(yaw))
    ey = hx*abs(np.sin(yaw))+hy*abs(np.cos(yaw))
    return np.stack([xy[:, 0]-ex-xmin, xmax-xy[:, 0]-ex,
                     xy[:, 1]-ey-ymin, ymax-xy[:, 1]-ey], axis=1)


def route_errors(fixture, run, xy):
    if not fixture.segments:
        return None, None
    pieces = []
    for origin, segment in zip(run["route_origins"], fixture.segments):
        pieces.extend(advance_pose(origin, segment.distance_m*u, segment.yaw_change_rad*u)[:2]
                      for u in np.linspace(0., 1., 251))
    path = np.asarray(pieces)
    errors = np.concatenate([
        np.sqrt(np.min(np.sum((chunk[:, None]-path[None, :])**2, axis=2), axis=1))
        for chunk in np.array_split(xy, max(1, math.ceil(len(xy)/128)))])
    return errors, float(np.sqrt(np.mean(errors**2)))


def summarize(fixture, run):
    rows = run["rows"]

    def col(name):
        return np.asarray([row[name] for row in rows])

    def event(name):
        return next((item["time_s"] for item in run["events"] if item["event"] == name), None)

    def sample_at(t):
        return int(np.searchsorted(time, t-1e-9))

    time = col("time_s")
    xy = np.column_stack([col("x_m"), col("y_m")])
    travel = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    speed = np.hypot(col("axle_v_mps"), col("axle_lateral_mps"))
    stopped = (speed <= .02) & (np.abs(col("body_yaw_rate_radps")) <= .05)
    effort = np.column_stack([col("left_drive_torque_Nm"), col("right_drive_torque_Nm")])
    phase = col("phase")
    positive = col("requested_v_mps") > 0.
    yaw_request = col("requested_yaw_rate_radps")
    motion_requested = (col("requested_v_mps") != 0.) | (yaw_request != 0.)
    qr_valid = col("qr_valid")
    qr = np.array([r["qr_error"] if r["qr_error"] is not None else np.nan for r in rows])
    moving_qr = positive & (phase == "advance")
    stop_s, stop_distance, stop_dwell_s = None, None, None
    braking_start_s = run["stop_phase_start_s"]
    if np.any(phase == "stop"):
        braking_start_s = float(time[np.flatnonzero(phase == "stop")[0]])
    if fixture.case == "M02" and fixture.segments and phase[-1] != "abort":
        braking_start_s = run["scheduled_duration_s"]
    elif (fixture.case in {"Q06", "Q13", "Q17"} or run["image_replay"]) and event("lost") is not None:
        braking_start_s = event("lost")
    elif fixture.case == "O03":
        braking_start_s = event("pedestrian")
    elif fixture.case == "O08":
        braking_start_s = event("link_down")
    if braking_start_s is not None:
        first = sample_at(braking_start_s)
        since = None
        for i in range(first, len(time)):
            if not stopped[i]:
                since = None
            elif since is None:
                since = i
            if since is not None and time[i]-time[since] >= .25-1e-9:
                stop_s = float(time[since]-time[first])
                stop_distance = float(travel[since]-travel[first])
                stop_dwell_s = float(time[i])
                break
    path_errors, path_rmse = route_errors(fixture, run, xy)
    for index, row in enumerate(rows):
        row["cross_track_error_m"] = float(path_errors[index]) if path_errors is not None else None
    caster = col("caster_angle_body_rad")
    longitudinal = col("axle_v_mps")
    caster_alignment = np.abs(wrap(caster-np.where(longitudinal < -.05, np.pi, 0.)))
    realignment = None
    motion_onset = np.flatnonzero(abs(longitudinal) >= .05)
    reversal = fixture.case == "M08"
    if reversal:
        # The outbound controller can briefly back up before the first gate.
        # The planned reverse phase begins after that gate's confirmed dwell.
        forward_gate = next((gate for gate in run["gates"] if gate["gate"] == 1), None)
        motion_onset = (np.flatnonzero((time > forward_gate["time_s"] + 1e-9) & (longitudinal < -.05))
                        if forward_gate else np.array([], dtype=int))
    elif fixture.case == "Q18":
        motion_onset = np.flatnonzero(longitudinal < -.05)
    if len(motion_onset):
        onset = motion_onset[0]
        stable = None
        for i in range(onset, len(time)):
            same_direction = (longitudinal[i] < -.05) if (reversal or fixture.case == "Q18") else (longitudinal[i] > .05)
            if not same_direction or caster_alignment[i] > math.radians(10):
                stable = None
            elif stable is None:
                stable = i
            if stable is not None and time[i]-time[stable] >= .25-1e-9:
                realignment = float(time[stable]-time[onset])
                break
    clearance = {}
    if fixture.case in {"M07", "O02", "O03", "Q18"}:
        yaw = col("yaw_rad")
        if fixture.case == "M07":
            bounds = ((-.44, .88, -.44, .92) if fixture.variant == "wider-envelope"
                      else (-.34, .74, -.36, .76))
        elif fixture.case == "O02":
            bounds = (-.35, .70, -.34, .34)
        elif fixture.case == "O03":
            bounds = (-.32, .91, -.34, .34)
        else:
            bounds = tuple(run["configuration"]["undock"]["wall_bounds_m"])
        clearances = wall_clearances(xy, yaw, bounds)
        wall = clearances.min(axis=1)
        worst = int(wall.argmin())
        failures = np.flatnonzero(wall <= 0.)
        clearance = {"minimum_wall_clearance_m": float(wall[worst]), "bounds_m": list(bounds),
                     "envelope_half_extents_m": [.25, .205],
                     "worst_at_s": float(time[worst]),
                     "worst_wall": ("xmin", "xmax", "ymin", "ymax")[clearances[worst].argmin()],
                     "first_violation_at_s": float(time[failures[0]]) if len(failures) else None}
        if fixture.case in {"M07", "Q18"}:
            for row, margin in zip(rows, wall, strict=True):
                row["wall_clearance_m"] = float(margin)
        if fixture.case == "M07":
            refs = np.column_stack([col("ref_x_m"), col("ref_y_m")])
            reference_wall = wall_clearances(refs, col("ref_yaw_rad"), bounds).min(axis=1)
            clearance["reference_sampled_minimum_clearance_m"] = float(reference_wall.min())
            for row, margin in zip(rows, reference_wall, strict=True):
                row["reference_wall_clearance_m"] = float(margin)
        if fixture.case == "O03":
            crossing = (time >= event("pedestrian")) & (time < event("clear"))
            rightmost = bounds[1]-clearances[:, 1]
            clearance["pedestrian_minimum_clearance_m"] = float(np.min(.52-rightmost[crossing]))
    gates = run["gates"]
    reason = run["termination_reason"]
    metrics = {
        "duration_s": float(time[-1]), "scheduled_reference_duration_s": run["scheduled_duration_s"],
        "completion_time_s": (gates[-1]["time_s"] if gates and reason == "route_gates_accepted"
                              else run["completed_s"]),
        "termination_reason": reason, "samples": len(rows), "gates": gates,
        "path_rmse_m": path_rmse, "final_position_error_m": rows[-1]["pose_error_m"],
        "final_yaw_error_rad": abs(rows[-1]["yaw_error_rad"]),
        "synchronized_position_rmse_m": float(np.sqrt(np.mean(col("pose_error_m")**2))) if fixture.segments else None,
        "synchronized_yaw_rmse_rad": float(np.sqrt(np.mean(col("yaw_error_rad")**2))) if fixture.segments else None,
        "yaw_travel_rad": float(np.sum(abs(np.diff(col("yaw_rad"))))),
        "max_axle_speed_mps": float(speed.max()), "final_axle_speed_mps": float(speed[-1]),
        "stop_trigger_time_s": braking_start_s, "stop_duration_s": stop_s,
        "stop_distance_m": stop_distance, "stop_dwell_verified_at_s": stop_dwell_s,
        "peak_torque_Nm": float(abs(effort).max()),
        "rms_left_torque_Nm": float(np.sqrt(np.mean(effort[:, 0]**2))),
        "rms_right_torque_Nm": float(np.sqrt(np.mean(effort[:, 1]**2))),
        "torque_saturated_sample_fraction": float(np.mean(np.max(abs(effort), axis=1) >= .5-1e-10)),
        "caster_peak_abs_swivel_rate_radps": float(np.max(abs(col("caster_relative_rate_radps")))),
        "caster_final_angle_rad": float(wrap(caster[-1])),
        "caster_realign_10deg_dwell_0_25s_after_motion_s": realignment,
        "caster_realign_status": ("reached" if realignment is not None else
                                  "not_reached_in_motion_window" if len(motion_onset) else "no_linear_motion"),
        "caster_lateral_slip_rms_mps": float(np.sqrt(np.mean(col("caster_lat_slip_mps")**2))),
        "drive_long_slip_rms_mps": float(np.sqrt(np.mean(np.r_[col("left_long_slip_mps"),col("right_long_slip_mps")]**2))),
        "mechanical_axle_work_J": rows[-1]["motor_work_J"],
        "energy_balance_max_abs_J": float(np.max(abs(col("energy_balance_error_J")))),
        "minimum_passive_dissipation_W": float(np.min(col("dissipation_power_W"))),
        "quasistatic_normal_load_N_at_final": [rows[-1][key] for key in ("normal_left_N","normal_right_N","normal_caster_N")],
        "axle_displacement_m": float(np.linalg.norm(xy[-1]-xy[0])),
        "max_drift_m": float(np.max(np.linalg.norm(xy-xy[0],axis=1))),
        "qr_first_error": float(qr[np.isfinite(qr)][0]) if np.isfinite(qr).any() else None,
        "qr_first_yaw_request_radps": float(col("requested_yaw_rate_radps")[np.isfinite(qr)][0]) if np.isfinite(qr).any() else None,
        "qr_first_advance_s": float(time[np.flatnonzero(moving_qr)[0]]) if np.any(moving_qr) else None,
        "qr_advance_peak_abs_error": float(np.max(abs(qr[moving_qr]))) if np.any(moving_qr) else None,
        "qr_invalid_forward_samples": int(np.count_nonzero(positive & ~qr_valid & (phase != "transit"))) if fixture.case.startswith("Q") else None,
        "qr_frame_rejections": {str(reason): int(np.count_nonzero(col("qr_reject_reason")==reason))
                                for reason in sorted(set(col("qr_reject_reason"))-{""})},
        "aligned": rows[-1]["aligned"], "contact": rows[-1]["contact"],
        "charge_confirmed": rows[-1]["charge_confirmed"], "retry_count": rows[-1]["retry_count"],
        "dock_final_position_error_m": rows[-1]["dock_geometry_error_m"],
        "dock_final_yaw_error_rad": rows[-1]["dock_geometry_yaw_error_rad"],
        "qr_yaw_sign_changes": int(np.count_nonzero(np.diff(np.sign(qr[np.isfinite(qr) & (abs(qr)>.005)])))),
        "clearance_geometry_proxy": clearance,
    }
    if fixture.segments:
        peak = max(progress(.5,segment_duration(seg,fixture),fixture.profile)[1]*abs(seg.distance_m) for seg in fixture.segments)
        peak_yaw = max(progress(.5,segment_duration(seg,fixture),fixture.profile)[1]*abs(seg.yaw_change_rad) for seg in fixture.segments)
        metrics.update(reference_peak_speed_mps=peak,reference_peak_yaw_rate_radps=peak_yaw)
    if reversal:
        metrics["reverse_band_onset_after_forward_gate_s"] = (
            float(time[motion_onset[0]]) if len(motion_onset) else None)
        metrics["caster_reverse_band_dwell_confirmed_s"] = (
            float(time[i]) if realignment is not None else None)
    if fixture.case == "M11":
        metrics["surface_breakdown"] = {
            surface: {"samples": int(mask.sum()), "peak_axle_speed_mps": float(speed[mask].max()),
                      "drive_long_slip_rms_mps": float(np.sqrt(np.mean(
                         np.r_[col("left_long_slip_mps")[mask],col("right_long_slip_mps")[mask]]**2)))}
            for surface in sorted(set(col("surface")))
            if np.any(mask := col("surface") == surface)}
    checks = {"finite_numeric": all(np.isfinite(value) for row in rows for value in row.values() if isinstance(value, (float, int))),
              "torque_limit": metrics["peak_torque_Nm"] <= .5,
              "passive_dissipation": metrics["minimum_passive_dissipation_W"] >= -1e-10,
              "energy_balance": metrics["energy_balance_max_abs_J"] < 1e-5}
    if fixture.segments:
        checks["terminal_gate"] = (reason == "route_gates_accepted" and phase[-1] == "settled"
                                   and bool(gates) and gates[-1]["gate"] == len(fixture.segments))
    if fixture.case == "M01":
        checks.update(zero_torque=metrics["peak_torque_Nm"] == 0., drift_below_1mm=metrics["max_drift_m"] <= .001)
    elif fixture.case == "M02":
        checks.update(stopped=stop_s is not None, forward_wheels=bool(col("left_wheel_rate_radps").max() > 0
                                                                  and col("right_wheel_rate_radps").max() > 0))
    elif fixture.case == "M04":
        checks["cross_track_rmse_at_most_3cm"] = path_rmse <= .03
    elif fixture.case in {"M05","M06"}:
        yaw = [item["target_pose"][2] for item in gates]
        signed_turns = np.diff(np.r_[0.,yaw])[1::2]
        checks.update(gate_order=[item["kind"] for item in gates] == ["edge","turn"]*4,
                      signed_90deg_turns=len(signed_turns)==4 and bool(np.allclose(
                          signed_turns, -np.pi/2 if fixture.case=="M05" else np.pi/2)))
    elif fixture.case == "M11":
        surface_change = event("surface_change")
        if run["configuration"]["surface_after_boundary"] is not None:
            checks["surface_crossed"] = surface_change is not None
        if (brake := event("brake_step")) is not None:
            brake_index = sample_at(brake)
            checks.update(braking_measured_from_command_cut=(0 < brake_index < len(time)
                          and braking_start_s == brake and positive[brake_index-1]
                          and not np.any(motion_requested[brake_index:]) and stop_s is not None),
                          brake_after_boundary=(run["configuration"]["surface_after_boundary"] is None
                                                or surface_change is not None and surface_change < brake))
            metrics["surface_brake"] = {"brake_command_s": brake, "surface_change_s": surface_change,
                                        "brake_surface": rows[brake_index]["surface"],
                                        "pre_brake_axle_speed_mps": float(speed[brake_index-1])}
    if run["configuration"]["camera"]:
        checks.update(zero_forward_when_invalid=not metrics["qr_invalid_forward_samples"],
                      forward_within_outer_band=not np.any(moving_qr & (np.abs(qr)>.04)),
                      image_feedback_signed=not np.any(qr_valid & (np.abs(qr)>.001) &
                          np.isin(phase, ["center","advance","recenter"]) & (qr*col("requested_yaw_rate_radps")>1e-12)))
    if fixture.case in {"Q01","Q02","Q03","Q15"}:
        checks.update(virtual_contact_without_charge=reason=="charge_timeout" and metrics["aligned"]
                      and metrics["contact"] and not metrics["charge_confirmed"])
    if fixture.case == "Q05":
        search_limit = run["configuration"]["qr_command_budgets"]["search_limit_s"]
        search = phase == "search"
        acquired_at = event("acquired")
        dt = np.diff(time)
        yaw = col("yaw_rad")
        boundary = np.flatnonzero(search)[-1]+1 if np.any(search) else 0
        search_end = min(boundary, len(time)-1)
        actual_excursion = yaw[:search_end+1]-yaw[0]
        metrics["search"] = {
            "first_acquisition_s": acquired_at,
            "end_s": float(time[search_end]),
            "duration_budget_s": search_limit,
            "yaw_request_peak_radps": float(np.max(np.abs(yaw_request[search]), initial=0.)),
            "yaw_request_absolute_integral_rad": float(np.sum(np.abs(yaw_request[:-1][search[:-1]])*dt[search[:-1]])),
            "achieved_yaw_excursion_rad": [float(actual_excursion.min()), float(actual_excursion.max())],
            "achieved_absolute_yaw_travel_rad": float(np.sum(np.abs(np.diff(yaw[:search_end+1])))),
            "achieved_peak_abs_yaw_rate_radps": float(np.max(np.abs(col("body_yaw_rate_radps")[:search_end+1]))),
        }
        checks.update(bounded_yaw_only_search=(not np.any(time[search] >= search_limit)
                      and not np.any(col("requested_v_mps")[search])
                      and np.all(np.abs(yaw_request[search]) <= Dock.SEARCH_YAW_LIMIT_RADPS)
                      and not np.any(search & motion_requested & (col("qr_reject_reason") != "missing"))),
                      fresh_detection_acquired_within_budget=(acquired_at is not None
                          and acquired_at < search_limit and qr_valid[sample_at(acquired_at)]),
                      safe_stop_verified=(stop_s is not None and run["completed_s"] is not None
                          and stopped[-1] and not np.any(motion_requested & (phase == "abort"))))
    if fixture.case in {"Q02","Q03"}:
        initial = metrics["qr_first_error"]
        heading = metrics["qr_first_yaw_request_radps"]
        checks["correct_initial_direction"] = initial is not None and (
            initial < 0 < heading if fixture.case=="Q02" else heading < 0 < initial)
    if fixture.case in {"Q06","Q13"}:
        checks.update(loss_stops_moving_robot=reason.startswith("observation_timeout:")
                      and event("lost") is not None and stop_s is not None and metrics["qr_first_advance_s"] is not None)
    if fixture.case == "Q13" and (rewind := event("capture_rewind")) is not None:
        rewind_index = sample_at(rewind)
        bad = rows[rewind_index]["qr_capture_time_s"]
        previous = rows[rewind_index-1]["qr_capture_time_s"]
        checks["fresh_regressive_capture_rejected"] = (bad is not None and previous is not None and bad < previous
            and rewind-bad <= .20 and rows[rewind_index]["qr_reject_reason"] == "disordered"
            and positive[rewind_index-1] and not np.any(motion_requested[rewind_index:]))
        metrics["capture_regression"] = {"first_regressive_arrival_s": rewind, "regressive_stamp_s": bad,
                                        "prior_accepted_stamp_s": previous}
    if fixture.case == "Q08":
        checks.update(wrong_identity_timeout=reason=="search_timeout:wrong_id",
                      did_not_move=metrics["max_drift_m"] <= .001)
    if fixture.case in {"Q07","Q16","O07"}:
        checks["virtual_charge_success"] = reason=="virtual_charge_confirmed" and metrics["charge_confirmed"]
        if fixture.case=="Q07":
            checks["reacquisition_recorded"] = metrics["retry_count"]==1
    if fixture.case=="Q14":
        obstacle_at = event("obstacle")
        checks.update(obstacle_stop=reason=="obstacle" and stop_s is not None,
                      no_motion_request_after_obstacle=obstacle_at is not None
                      and not np.any(motion_requested & (time >= obstacle_at)))
    if fixture.case=="Q17":
        checks["retry_budget_aborts"] = reason=="retry_budget" and metrics["retry_count"]==3
    if fixture.case == "Q04" and "staging" in run["configuration"]:
        cfg = run["configuration"]["staging"]
        staged_s = event("staging_complete")
        checks.update(staging_observed_and_qualified=staged_s is not None and len(cfg["gates"]) == 3,
                      final_mechanical_pose_qualified=(metrics["aligned"] and metrics["contact"]
                          and metrics["dock_final_position_error_m"] <= Dock.POSITION_LIMIT_M
                          and metrics["dock_final_yaw_error_rad"] <= Dock.YAW_LIMIT_RAD))
        metrics["staged_s"] = staged_s
    if fixture.case in {"Q04","Q19","Q20"}:
        checks["docking_terminated_safely"] = run["completed_s"] is not None and stopped[-1]
    if fixture.case == "Q20":
        checks["settled_center_and_advanced"] = metrics["qr_first_advance_s"] is not None
    if run["image_replay"]:
        captures = run["image_replay"].rows
        first_capture = captures[0] if captures else None
        first_arrival = first_capture["arrival_time_s"] if first_capture else None
        image_control = col("qr_frame_source") == "synthetic_image_replay"
        selected = col("qr_selected_id")
        image_samples = np.flatnonzero(image_control)
        first_image_control = int(image_samples[0]) if len(image_samples) else None
        expected_valid = run["configuration"]["image_replay"]["fixture_expected_decoder_valid"]
        metrics["image_handoff"] = {
            "first_capture_s": first_capture["capture_time_s"] if first_capture else None,
            "first_arrival_s": first_arrival,
            "first_control_sample_s": float(time[first_image_control]) if first_image_control is not None else None,
            "first_control_reason": rows[first_image_control]["reason"] if first_image_control is not None else None,
            "first_requested_v_mps": float(col("requested_v_mps")[first_image_control])
            if first_image_control is not None else None,
            "decoder_captures": len(captures), "decoder_valid_captures": sum(item["decoder_valid"] for item in captures),
            "decoder_wall_median_s": float(np.median([item["decode_wall_s"] for item in captures])) if captures else None,
            "first_selected_id": selected[first_image_control] if first_image_control is not None else None,
        }
        checks.update(image_injected_while_approaching=first_image_control is not None
                      and first_image_control > 0 and positive[first_image_control-1],
                      first_image_consumed_on_first_possible_control_sample=first_arrival is not None
                      and first_image_control == sample_at(first_arrival),
                      decoder_and_gate_match_expected_validity=bool(len(image_samples)) and
                      all(item["decoder_valid"] == expected_valid for item in captures)
                      and bool(np.all(qr_valid[image_control] == expected_valid)),
                      correct_selection_or_immediate_interlock=bool(np.all(selected[image_control] == DOCK_ID))
                      if expected_valid else stop_s is not None and not np.any(motion_requested & image_control),
                      no_charge_confirmation_from_image=not np.any(col("charge_confirmed")))
        if fixture.case == "Q09" and fixture.camera_image_fixture == "mixed_ids":
            checks["distinct_candidates_seen"] = all(item["candidate_count"] == 2 for item in captures)
        if fixture.case == "Q09" and fixture.camera_image_fixture == "duplicate_target":
            checks["duplicate_id_interlock"] = bool(np.all(col("qr_reject_reason")[image_control] == "ambiguous_id"))
    if fixture.case == "Q18":
        authorization = event("undock_authorization")
        release = event("virtual_release_accepted")
        reverse = event("backout")
        reverse_samples = col("requested_v_mps") < 0.
        budget = run["configuration"]["undock"]
        measured_backout = float(xy[0, 0]-xy[-1, 0])
        metrics["undock"] = {"authorization_s": authorization, "release_s": release,
                             "first_reverse_s": reverse, "backout_request_end_s": event("stop"),
                             "axle_backout_m": measured_backout, "release_source": budget["release_source"]}
        checks.update(authorization_then_release_then_reverse=authorization is not None and release is not None
                      and reverse is not None and authorization < release < reverse
                      and not np.any(motion_requested & (time <= release)),
                      rejected_pre_authorization_release=event("virtual_release_rejected") is not None,
                      reverse_without_virtual_contact_or_charge=bool(np.any(reverse_samples))
                      and not np.any(reverse_samples & (col("undock_virtual_charging") | col("undock_virtual_contact"))),
                      bounded_backout_and_stop=reason == "backout_distance_reached" and stop_s is not None
                      and measured_backout >= budget["reverse_distance_m"] and not np.any(positive)
                      and not np.any(yaw_request))
    if fixture.case=="O03":
        hold = (time >= event("pedestrian")) & (time < event("authorized_resume"))
        checks.update(stop_and_authorized_resume=stop_s is not None and not np.any(positive & hold)
                      and np.any(positive & (time > event("authorized_resume"))),
                      pedestrian_geometry_clear=clearance["pedestrian_minimum_clearance_m"] > 0)
    if clearance:
        checks["wall_geometry_clear"] = clearance["minimum_wall_clearance_m"] > 0.
    if fixture.case == "O04" and run["configuration"]["command_input"]:
        trigger = event("guard_stop")
        trigger_index = sample_at(trigger) if trigger is not None else len(time)
        accepted = np.flatnonzero(col("stream_accepted"))
        last = float(time[accepted[-1]]) if len(accepted) else None
        publisher_cut = event("publisher_cut")
        timeout = run["configuration"]["command_input"]["command_timeout_s"]
        expected_reason = "command_timeout" if publisher_cut is not None else "operator_abort"
        dt = time[1]-time[0]
        detected = ((last is not None and trigger is not None and
                     timeout-1e-10 <= trigger-last <= timeout+dt+1e-10)
                    if publisher_cut is not None else trigger == event("stream_operator_cancel"))
        metrics["stream_stop"] = {"last_accepted_request_s": last, "publisher_cut_s": publisher_cut,
                                  "guard_stop_s": trigger, "command_timeout_s": timeout}
        checks.update(guard_detected_fault=detected and reason == expected_reason,
                      immediate_zero_and_planar_stop=stop_s is not None and not np.any(motion_requested[trigger_index:]))
    elif fixture.case == "O05" and run["configuration"]["command_input"]:
        trigger = event("guard_stop")
        trigger_index = sample_at(trigger) if trigger is not None else len(time)
        cut = event("odometry_stream_cut")
        rewind = event("odometry_stamp_rewind")
        timeout = run["configuration"]["command_input"]["odometry_timeout_s"]
        sample = rows[trigger_index] if 0 < trigger_index < len(rows) else {}
        supplied_stamp = sample.get("stream_odom_frame_stamp_s")
        last_stamp = sample.get("stream_odom_last_valid_stamp_s")
        dt = float(time[1]-time[0])
        if cut is not None:
            detected = (supplied_stamp is None and last_stamp is not None and trigger is not None
                        and last_stamp < cut <= trigger
                        and timeout-1e-10 <= trigger-last_stamp <= timeout+dt+1e-10)
            expected_reason = "odom_timeout"
        else:
            detected = (rewind is not None and rewind == trigger and supplied_stamp is not None
                        and last_stamp is not None and supplied_stamp < last_stamp)
            expected_reason = "odom_regression"
        metrics["odometry_guard_stop"] = {"outage_requested_s": cut, "rewind_requested_s": rewind,
                                          "last_valid_odometry_stamp_s": last_stamp,
                                          "supplied_odometry_stamp_s": supplied_stamp,
                                          "first_zero_command_s": trigger, "expected_reason": expected_reason,
                                          "watchdog_s": timeout}
        checks.update(guard_detected_odometry_fault=detected and reason == expected_reason,
                      drove_before_fault=trigger_index > 0 and bool(motion_requested[trigger_index-1]),
                      immediate_zero_and_planar_stop=stop_s is not None and not np.any(motion_requested[trigger_index:]))
    elif fixture.case in {"O04","O05"}:
        trigger = next((item for item in run["events"] if item["reason"] == "injected_stop_trigger"), None)
        observed = trigger is not None and trigger["event"] in STOP_INTERLOCKS and reason == trigger["event"]
        checks.update(injected_stop_trigger_observed=observed,
                      post_trigger_braking_dwell=(observed and stop_s is not None
                          and not np.any(motion_requested[time >= trigger["time_s"]])))
    if fixture.case == "O08":
        down, restored = event("link_down"), event("link_restored")
        restart, authorization = event("guard_restart"), event("fresh_authorization")
        stale = event("stale_epoch_request")
        reopened = np.flatnonzero(motion_requested & (time > authorization)) if authorization is not None else []
        first = float(time[reopened[0]]) if len(reopened) else None
        stale_row = rows[sample_at(stale)] if stale is not None else None
        metrics["stream_recovery"] = {"link_down_s": down, "link_restored_s": restored,
                                      "reset_s": restart, "authorization_s": authorization,
                                      "stale_arrival_s": stale, "first_motion_request_s": first}
        checks.update(zero_through_restart_and_authorization=down is not None and authorization is not None
                      and bool(np.any(positive & (time < down)))
                      and not np.any(motion_requested & (time >= down) & (time <= authorization)),
                      prior_epoch_command_rejected=(stale_row is not None and stale_row.get("stream_input_epoch") is not None
                                                    and stale_row["stream_input_epoch"] < stale_row["stream_epoch"]
                                                    and not stale_row["stream_forwarded"]
                                                    and not motion_requested[sample_at(stale)]),
                      motion_resumed_on_new_authorized_request=first is not None and
                      bool(rows[reopened[0]]["stream_accepted"]),
                      planar_stop_dwell_before_resume=stop_dwell_s is not None and first is not None
                      and stop_dwell_s < first)
    metrics["checks"] = {k: bool(v) for k,v in checks.items()}
    return metrics
