"""Compute run-level acceptance and record measurable scope gaps."""
import math

import numpy as np

from scenarios import progress, segment_duration
from twin import advance_pose, wrap


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
    qr_valid = col("qr_valid")
    qr = np.array([r["qr_error"] if r["qr_error"] is not None else np.nan for r in rows])
    moving_qr = positive & (phase == "advance")
    stop_s, stop_distance, stop_dwell_s = None, None, None
    braking_start_s = run["stop_phase_start_s"]
    if np.any(phase == "stop"):
        braking_start_s = float(time[np.flatnonzero(phase == "stop")[0]])
    if fixture.case == "M02" and fixture.segments:
        braking_start_s = run["scheduled_duration_s"]
    elif fixture.case in {"Q06", "Q13", "Q17"} and event("lost") is not None:
        braking_start_s = event("lost")
    elif fixture.case == "O03":
        braking_start_s = event("pedestrian")
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
    if len(motion_onset):
        # Measure reorientation under reversal from the first sustained backward travel.
        reverse_onset = np.flatnonzero(longitudinal < -.05) if fixture.case == "M08" else []
        onset = reverse_onset[0] if len(reverse_onset) else motion_onset[0]
        stable = None
        for i in range(onset, len(time)):
            same_direction = (longitudinal[i] < -.05) if len(reverse_onset) else (longitudinal[i] > .05)
            if not same_direction or caster_alignment[i] > math.radians(10):
                stable = None
            elif stable is None:
                stable = i
            if stable is not None and time[i]-time[stable] >= .25-1e-9:
                realignment = float(time[stable]-time[onset])
                break
    clearance = {}
    if fixture.case in {"M07", "O02", "O03"}:
        yaw = col("yaw_rad")
        # Conservative envelope includes the tire width outside the 0.32 m chassis.
        hx, hy = .25, .205
        if fixture.case == "M07":
            xmin, xmax, ymin, ymax = ((-.44, .88, -.44, .92) if fixture.variant == "wider-envelope"
                                     else (-.34, .74, -.36, .76))
        elif fixture.case == "O02":
            xmin, xmax, ymin, ymax = -.35, .70, -.34, .34
        else:
            xmin, xmax, ymin, ymax = -.32, .91, -.34, .34
        ex = hx*abs(np.cos(yaw))+hy*abs(np.sin(yaw))
        ey = hx*abs(np.sin(yaw))+hy*abs(np.cos(yaw))
        wall = np.min(np.stack([xy[:, 0]-ex-xmin, xmax-xy[:, 0]-ex,
                               xy[:, 1]-ey-ymin, ymax-xy[:, 1]-ey]), axis=0)
        clearance = {"minimum_wall_clearance_m": float(wall.min()), "bounds_m": [xmin,xmax,ymin,ymax]}
        if fixture.case == "O03":
            crossing = (time >= event("pedestrian")) & (time < event("clear"))
            clearance["pedestrian_minimum_clearance_m"] = float(np.min(.52-(xy[:, 0]+ex)[crossing]))
    gates = run["gates"]
    reason = run["termination_reason"]
    metrics = {
        "duration_s": float(time[-1]), "scheduled_reference_duration_s": run["scheduled_duration_s"],
        "completion_time_s": gates[-1]["time_s"] if gates else run["completed_s"],
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
        checks["terminal_gate"] = reason == "route_gates_accepted"
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
    elif fixture.case == "M11" and fixture.variant == "boundary":
        checks["surface_crossed"] = event("surface_change") is not None
    if fixture.case.startswith("Q"):
        checks.update(zero_forward_when_invalid=not metrics["qr_invalid_forward_samples"],
                      forward_within_outer_band=not np.any(moving_qr & (np.abs(qr)>.04)),
                      image_feedback_signed=not np.any(qr_valid & (np.abs(qr)>.001) &
                          np.isin(phase, ["center","advance","recenter"]) & (qr*col("requested_yaw_rate_radps")>1e-12)))
    if fixture.case in {"Q01","Q02","Q03","Q05","Q15"}:
        checks.update(virtual_contact_without_charge=reason=="charge_timeout" and metrics["aligned"]
                      and metrics["contact"] and not metrics["charge_confirmed"])
    if fixture.case in {"Q02","Q03"}:
        initial = metrics["qr_first_error"]
        heading = metrics["qr_first_yaw_request_radps"]
        checks["correct_initial_direction"] = initial is not None and (
            initial < 0 < heading if fixture.case=="Q02" else heading < 0 < initial)
    if fixture.case in {"Q06","Q13"}:
        checks.update(loss_stops_moving_robot=reason.startswith("observation_timeout:")
                      and event("lost") is not None and stop_s is not None and metrics["qr_first_advance_s"] is not None)
    if fixture.case == "Q08":
        checks.update(wrong_identity_timeout=reason=="search_timeout:wrong_id",
                      did_not_move=metrics["max_drift_m"] <= .001)
    if fixture.case in {"Q07","Q16","O07"}:
        checks["virtual_charge_success"] = reason=="virtual_charge_confirmed" and metrics["charge_confirmed"]
        if fixture.case=="Q07":
            checks["reacquisition_recorded"] = metrics["retry_count"]==1
    if fixture.case=="Q14":
        checks.update(obstacle_stop=reason=="obstacle" and stop_s is not None,
                      no_motion_request_after_obstacle=not np.any(positive & (time >= event("obstacle"))))
    if fixture.case=="Q17":
        checks["retry_budget_aborts"] = reason=="retry_budget" and metrics["retry_count"]==3
    if fixture.case in {"Q04","Q19","Q20"}:
        checks["docking_terminated_safely"] = run["completed_s"] is not None and stopped[-1]
    if fixture.case == "Q20":
        checks["settled_center_and_advanced"] = metrics["qr_first_advance_s"] is not None
    if fixture.case=="O03":
        hold = (time >= event("pedestrian")) & (time < event("authorized_resume"))
        checks.update(stop_and_authorized_resume=stop_s is not None and not np.any(positive & hold)
                      and np.any(positive & (time > event("authorized_resume"))),
                      pedestrian_geometry_clear=clearance["pedestrian_minimum_clearance_m"] > 0)
    if clearance:
        checks["wall_geometry_clear"] = clearance["minimum_wall_clearance_m"] > 0.
    if fixture.case in {"O04","O05"}:
        checks.update(fault_latched=reason in {"operator_cancel","command_timeout","odometry_timeout","odometry_clock_regression"},
                      stopped=stop_s is not None and not np.any(positive & (phase == "abort")))
    metrics["checks"] = {k: bool(v) for k,v in checks.items()}
    return metrics
