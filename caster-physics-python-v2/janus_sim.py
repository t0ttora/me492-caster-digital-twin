"""Run the V2.3 RK4 plant with JANUS task controllers and time-aligned logging."""
from dataclasses import asdict, replace
from heapq import heappop, heappush
from math import atan2, cos, hypot, sin

import numpy as np

from janus_command_input import CommandInput
from janus_image_input import ImageReplay
from janus_undock import Undock
from qr_camera import (Camera, DOCK_ID, Frame, Marker, estimate_marker,
                       marker_pose_in_camera, mechanical_errors, project)
from qr_docking import Dock, FrameGate
from scenarios import Dwell, Route, STOP_INTERLOCKS, axle_state, terminal_ok
from twin import (Segment, State, Torques, TrackingController, case_parameters, evaluate_motion,
                  kinetic_energy, result_record, rk4_step, validate_parameters, wrap)

LINEAR_COMMAND_LIMIT_MPS = .25
YAW_COMMAND_LIMIT_RADPS = 1.


def parameters_for(fixture):
    p = case_parameters(fixture.surface)
    if fixture.payload_kg:
        # Lump the payload at nominal body COM; use its synthetic box yaw inertia.
        p = replace(p, dynamics=replace(p.dynamics,
                    body_lumped_mass_kg=p.dynamics.body_lumped_mass_kg+fixture.payload_kg,
                    body_yaw_inertia_kgm2=p.dynamics.body_yaw_inertia_kgm2
                    +fixture.payload_kg*(.24**2+.18**2)/12))
    validate_parameters(p)
    return p


def initial_state(fixture, p):
    yaw = np.deg2rad(fixture.yaw_deg)
    cx, cy = p.dynamics.body_com_m[:2]
    return replace(State.at_rest(p), body_x_m=fixture.axle_x_m+cos(yaw)*cx-sin(yaw)*cy,
                   body_y_m=fixture.y_m+sin(yaw)*cx+cos(yaw)*cy, body_yaw_rad=yaw,
                   caster_angle_relative_rad=np.deg2rad(fixture.caster_deg))


class CameraStream:
    DEFAULT_NOISY_DELAYS_S = (.07, .09)

    def __init__(self, camera, marker, seed, noisy, delays_s=(), image_replay=None):
        self.camera, self.marker = camera, marker
        self.rng = np.random.default_rng(seed) if noisy else None
        self.delays_s = delays_s or (self.DEFAULT_NOISY_DELAYS_S if noisy else (0.,))
        self.image_replay = image_replay
        self.queue = []
        self.frames = []
        self.last = None

    def sample(self, index, time_s, dt, pose, mode):
        frame_ticks = round(self.camera.frame_period_s/dt)
        if index % frame_ticks == 0 and mode != "stream_cut":
            delay = self.delays_s[index//frame_ticks % len(self.delays_s)]
            if mode == "stale":
                delay = .27
            if mode == "image_replay":
                frame = self.image_replay.capture(time_s, self.camera)
            elif mode == "duplicate" and self.last is not None:
                frame = replace(self.last, arrival_time_s=time_s)
            elif mode == "capture_rewind" and self.last is not None:
                capture = round(max(0., self.last.capture_time_s - dt), 9)
                frame = replace(self.last, capture_time_s=capture, arrival_time_s=time_s)
            else:
                identity = "JANUS-OTHER-99" if mode == "wrong" else DOCK_ID
                candidate = None if mode == "loss" else project(pose, self.camera, self.marker, identity, self.rng)
                candidates = (candidate,) if candidate and candidate.visible else ()
                frame = Frame(time_s, round(time_s+delay, 9), candidates)
            heappush(self.queue, (frame.arrival_time_s, index, frame))
            self.last = frame
        arrivals = []
        while self.queue and self.queue[0][0] <= time_s+1e-10:
            _, _, frame = heappop(self.queue)
            arrivals.append(frame)
            self.frames.append(frame)
        return arrivals


QR_STAGING_STANDOFF_M = .55


def qr_staging_route(fixture, parameters, axle_pose, camera, observation, side_m):
    """Use the observed marker transform to plan a verified pre-dock staging pose."""
    forward, lateral, camera_heading = marker_pose_in_camera(observation.candidate, camera, side_m)
    yaw = axle_pose[2] + camera.yaw_rad
    approach_yaw = yaw + camera_heading - np.deg2rad(fixture.marker_mount_calibration_deg)
    camera_world = axle_pose[:2] + np.array([
        cos(axle_pose[2])*camera.forward_m-sin(axle_pose[2])*camera.left_m,
        sin(axle_pose[2])*camera.forward_m+cos(axle_pose[2])*camera.left_m,
    ])
    station_xy = camera_world + np.array([
        forward*cos(yaw)-lateral*sin(yaw),
        forward*sin(yaw)+lateral*cos(yaw),
    ])
    staging_xy = station_xy - QR_STAGING_STANDOFF_M*np.array([cos(approach_yaw), sin(approach_yaw)])
    dx, dy = staging_xy - axle_pose[:2]
    bearing = atan2(dy, dx)
    distance = hypot(dx, dy)
    segments = (Segment(0., float(wrap(bearing-axle_pose[2])), 1.3),
                Segment(distance, 0., max(2., distance/.10)),
                Segment(0., float(wrap(approach_yaw-bearing)), 1.6))
    return Route(replace(fixture, segments=segments, gate_each=True, profile="quintic"), parameters, axle_pose)


def simulate(fixture):
    p = parameters_for(fixture)
    step = p.dynamics.solver_step_s
    dt = p.dynamics.command_sample_dt_s
    steps_per_command = round(dt/step)
    assert abs(dt/step-steps_per_command) < 1e-9 and p.dynamics.result_sample_dt_s == dt
    start = initial_state(fixture, p)
    vector = start.as_vector()
    initial_energy = kinetic_energy(start, p)
    initial_pose = axle_state(start, p)[0]
    route = Route(fixture, p, initial_pose) if fixture.segments else None
    undock = Undock(initial_pose[0]) if fixture.case == "Q18" else None
    use_camera = (fixture.case.startswith("Q") and not undock) or fixture.case == "O07"
    event_types = {event for _, event in fixture.events}
    command_input = CommandInput(recovery=fixture.case == "O08") if "control_stream" in event_types else None
    camera = Camera(left_m=.035, yaw_rad=np.deg2rad(4), target_u_px=510.*np.tan(np.arctan2(.035, .22)-np.deg2rad(4))+320.) if "camera_offset" in event_types else Camera()
    dock_reference = Marker()
    marker = replace(dock_reference, yaw_rad=np.deg2rad(fixture.marker_mount_yaw_deg))
    image_replay = ImageReplay(fixture.case, fixture.camera_image_fixture) if fixture.camera_image_fixture else None
    camera_stream = CameraStream(camera, marker, fixture.seed, "delay_noise" in event_types,
                                 fixture.camera_delays_s, image_replay)
    frame_gate = FrameGate(camera)
    observation = frame_gate.current
    dock = Dock(camera, marker.side_m, fixture.qr_yaw_gain, fixture.search_sweep,
                search_enabled=fixture.case == "Q05", search_limit_s=fixture.search_limit_s,
                search_reverse_s=fixture.search_reverse_s) if use_camera else None
    wheel = TrackingController(p, "straight", "baseline")
    transition_events, rows = [], []
    schedule = sorted(fixture.events)
    motion_stop = Dwell(.25)
    stop_phase_start = None
    mode, interlock, pedestrian_hold = "visible", None, False
    transit = "transit" in event_types
    braking = False
    route_goal = initial_pose.copy()
    reason = "duration_limit"
    high_drag = case_parameters("high_drag_synthetic") if "surface_boundary_x_0.30" in event_types else None
    step_parameters = p
    completed_s = None
    last_bms_s = None
    staging = None
    staging_active = "qr_pose_staging" in event_types
    staging_complete = False
    staging_start_s = None
    for tick in range(round(fixture.duration_s/dt)+1):
        t = round(tick*dt, 9)
        state = State.from_vector(vector)
        pose, v, lateral = axle_state(state, p)
        stopped = terminal_ok((0., 0., hypot(v, lateral), abs(state.body_yaw_rate_radps)))
        while schedule and schedule[0][0] <= t+1e-10:
            _, event = schedule.pop(0)
            injection = fixture.case in {"O04", "O05"} and event in STOP_INTERLOCKS
            transition_events.append(dict(time_s=t, event=event,
                                          reason="injected_stop_trigger" if injection else "fixture"))
            if event in {"visible", "wrong", "loss", "stale", "duplicate", "capture_rewind",
                         "stream_cut", "image_replay"}:
                mode = event
            elif event in STOP_INTERLOCKS and interlock is None:
                interlock = event
            elif event == "pedestrian":
                pedestrian_hold = True
            elif event == "authorized_resume":
                pedestrian_hold = False
            elif event == "low_soc":
                transit = False
            elif event == "brake_step":
                braking = True
            if command_input:
                command_input.handle_event(t, event)
            if undock and event == "undock_authorization":
                undock.authorize(t)
            elif undock and event == "virtual_release_frame":
                accepted = undock.release(t, t)
                transition_events.append(dict(time_s=t, event="virtual_release_accepted" if accepted
                                              else "virtual_release_rejected", reason="synthetic_bms_and_contact"))
        if high_drag is not None and step_parameters is p and pose[0] >= .30:
            step_parameters = high_drag
            transition_events.append(dict(time_s=t, event="surface_change", reason="axle_crossed_boundary"))
        estimator_range, estimator_heading = None, None
        current_bms_s, mechanical = None, (None, None)
        if use_camera:
            arrivals = camera_stream.sample(tick, t, dt, pose, mode)
            observation = frame_gate.update(t, arrivals)
            mechanical = mechanical_errors(pose, dock_reference, Dock.STANDOFF_AXLE_M)
            if observation.valid:
                estimator_range, estimator_heading = estimate_marker(observation.candidate, camera, marker.side_m)
            route_goal = np.array([dock_reference.x_m-Dock.STANDOFF_AXLE_M, dock_reference.y_m, dock_reference.yaw_rad])
        if dock and not transit and interlock is None:
            # An interrupted docking attempt must not resume a historical contact hold.
            if pedestrian_hold:
                interlock = "pedestrian_interlock_latched"
            elif braking:
                interlock = "brake_step"
        stream_row = {}
        if undock:
            previous = undock.phase
            requested_v, requested_w, why = undock.update(t, pose[0], stopped, interlock)
            phase = undock.phase
            route_goal = initial_pose - np.array([undock.budget.reverse_distance_m, 0., 0.])
            if phase != previous:
                transition_events.append(dict(time_s=t, event=phase, reason=why))
        elif interlock:
            requested_v, requested_w, why = dock.abort(t, interlock) if dock else (0., 0., interlock)
            phase = "abort"
        elif pedestrian_hold:
            requested_v, requested_w, phase, why = 0., 0., "hold", "pedestrian_interlock_latched"
        elif braking or (fixture.case == "O03" and pose[0] >= .54):
            braking = True
            why = "transit_target" if fixture.case == "O03" else "brake_step"
            requested_v, requested_w, phase = 0., 0., "stop"
        elif staging_active and not staging_complete:
            requested_v, requested_w, phase, why = 0., 0., "stage_wait", "await_fresh_marker_pose"
            if staging is None and observation.valid:
                staging = qr_staging_route(fixture, p, pose, camera, observation, marker.side_m)
                staging_start_s = t
                transition_events.append(dict(time_s=t, event="staging_plan", reason="projected_corners"))
            if staging is not None:
                if not observation.valid:
                    interlock = "staging_marker_lost"
                    requested_v, requested_w, why = dock.abort(t, interlock)
                    phase = "abort"
                else:
                    requested_v, requested_w, route_goal, step_phase = staging.update(
                        t-staging_start_s, pose, v, lateral, state.body_yaw_rate_radps)
                    phase, why = "staging", step_phase
                    if step_phase == "settled":
                        dock.search_started_s = t
                        staging_complete = True
                        transition_events.append(dict(time_s=t, event="staging_complete", reason="pose_velocity_dwell"))
            elif t >= Dock.SEARCH_LIMIT_S:
                interlock = "staging_marker_unavailable"
                requested_v, requested_w, why = dock.abort(t, interlock)
                phase = "abort"
        elif route:
            requested_v, requested_w, route_goal, phase = route.update(t, pose, v, lateral, state.body_yaw_rate_radps)
            why = phase
        elif command_input:
            guarded, stream_row = command_input.update(tick, t, dt)
            requested_v, requested_w, why = guarded.v_mps, guarded.yaw_rate_radps, guarded.reason
            phase = "transit" if why == "ready" else ("abort" if fixture.case in {"O04", "O05"} else "hold")
            if phase == "abort" and stop_phase_start is None:
                transition_events.append(dict(time_s=t, event="guard_stop", reason=why))
        elif use_camera and not transit:
            if ("virtual_charge" in event_types and dock.phase == "wait_for_charge" and last_bms_s is None
                    and t-dock.wait_s >= .30-1e-10):
                last_bms_s = current_bms_s = t
                transition_events.append(dict(time_s=t, event="virtual_bms_frame", reason="synthetic_charge_source"))
            final_yaw_error = None
            if fixture.trim_dock_yaw and observation.valid:
                final_yaw_error = float(wrap(estimator_heading + camera.yaw_rad
                                            - np.deg2rad(fixture.marker_mount_calibration_deg)))
            requested_v, requested_w, why = dock.update(t, observation, stopped, mechanical,
                bms_time_s=current_bms_s, final_yaw_error_rad=final_yaw_error)
            phase = dock.phase
        elif fixture.case == "M01":
            requested_v, requested_w, phase = 0., 0., "idle"
            why = "zero_torque_fixture"
        else:
            requested_v, requested_w, phase, why = .14, 0., "transit", "fixture_velocity_request"
        limited_v = float(np.clip(requested_v, -LINEAR_COMMAND_LIMIT_MPS, LINEAR_COMMAND_LIMIT_MPS))
        limited_w = float(np.clip(requested_w, -YAW_COMMAND_LIMIT_RADPS, YAW_COMMAND_LIMIT_RADPS))
        geometry = p.geometry
        wheel_targets = (np.array([limited_v-geometry.drive_track_m*limited_w/2,
                                   limited_v+geometry.drive_track_m*limited_w/2])/geometry.drive_radius_m)
        if fixture.case == "M01":
            torque = Torques(0., 0.)
        else:
            if phase != (rows[-1]["phase"] if rows else "") and not (limited_v or limited_w):
                wheel.integral_torque_Nm[:] = 0.
            torque = wheel.wheel_torques(state, wheel_targets, dt)
        row = result_record(t, state, torque, step_parameters,
                            evaluate_motion(state, torque, step_parameters), initial_energy)
        row.update(run_id=f"{fixture.case}-python-{fixture.variant}", phase=phase, reason=why,
                   requested_v_mps=requested_v, requested_yaw_rate_radps=requested_w,
                   limited_v_mps=limited_v, limited_yaw_rate_radps=limited_w,
                   axle_v_mps=v, axle_lateral_mps=lateral,
                   ref_x_m=route_goal[0], ref_y_m=route_goal[1], ref_yaw_rad=route_goal[2],
                   pose_error_m=float(np.linalg.norm(pose[:2]-route_goal[:2])),
                   yaw_error_rad=float(wrap(pose[2]-route_goal[2])),
                   gate_index=len(route.gates) if route else 0,
                   surface="high_drag_synthetic" if step_parameters is high_drag else fixture.surface,
                   qr_error=observation.error, qr_valid=observation.valid,
                   qr_reject_reason="" if observation.valid or not use_camera else observation.reason,
                   qr_capture_time_s=observation.frame.capture_time_s if observation.frame else None,
                   qr_arrival_time_s=observation.frame.arrival_time_s if observation.frame else None,
                   qr_range_estimate_m=estimator_range, qr_station_yaw_relative_rad=estimator_heading,
                   dock_geometry_error_m=mechanical[0], dock_geometry_yaw_error_rad=mechanical[1],
                   aligned=dock.aligned if dock else False, contact=dock.contact if dock else False,
                   charge_confirmed=dock.charge_confirmed if dock else False,
                   bms_capture_time_s=last_bms_s, retry_count=dock.retries if dock else 0)
        row.update(stream_row)
        if image_replay:
            row.update(qr_frame_source=observation.frame.source if observation.frame else None,
                       qr_selected_id=observation.candidate.id if observation.valid else None)
        if undock:
            row.update(undock_virtual_charging=undock.charging, undock_virtual_contact=undock.contact,
                       undock_authorized_s=undock.authorized_s, undock_release_s=undock.release_s)
        rows.append(row)
        if phase == "abort":
            reason = why
        finished = (phase in {"abort", "settled"} or (dock and dock.done)
                    or (undock and undock.done)
                    or (phase == "stop" and braking))
        if finished:
            if stop_phase_start is None:
                stop_phase_start = t
            if motion_stop.update(t, stopped):
                completed_s = t
                reason = "route_gates_accepted" if phase == "settled" else why
                break
        else:
            motion_stop.update(t, False)
            stop_phase_start = None
        if tick == round(fixture.duration_s/dt):
            if fixture.case == "M01" and phase == "idle":
                reason = "idle_window_elapsed"
            break
        for _ in range(steps_per_command):
            vector = rk4_step(vector, step, lambda values:
                              evaluate_motion(State.from_vector(values), torque, step_parameters).derivative)
    gates = route.gates if route else []
    events = transition_events + (dock.events if dock else []) + [
        dict(time_s=item["time_s"], event="gate_accepted", reason=f"{item['gate']}_{item['kind']}") for item in gates]
    events.sort(key=lambda item: item["time_s"])
    config = {"parameters": asdict(p), "surface_after_boundary": asdict(high_drag) if high_drag else None,
              "camera": asdict(camera) if use_camera else None, "marker": asdict(marker) if use_camera else None,
              "velocity_limits": {"max_abs_v_mps": LINEAR_COMMAND_LIMIT_MPS,
                                  "max_abs_yaw_radps": YAW_COMMAND_LIMIT_RADPS},
              "target_axle_standoff_m": Dock.STANDOFF_AXLE_M if use_camera else None,
              "qr_command_budgets": dock.command_budgets() if dock else None,
              "camera_transport": {"delays_s": camera_stream.delays_s,
                                   "corner_noise_sigma_px": .9 if camera_stream.rng is not None else 0.}
              if use_camera else None,
              "image_replay": image_replay.configuration() if image_replay else None,
              "command_input": command_input.budgets() if command_input else None,
              "undock": undock.configuration() if undock else None}
    if staging_active:
        config["staging"] = {"observed_marker_standoff_m": QR_STAGING_STANDOFF_M,
                             "start_s": staging_start_s,
                             "target_pose": staging.origins[-1].tolist() if staging else None,
                             "gates": staging.gates if staging else []}
    if fixture.marker_mount_yaw_deg:
        config["marker_mount"] = {"actual_marker_from_dock_yaw_deg": fixture.marker_mount_yaw_deg,
                                  "configured_marker_from_dock_yaw_deg": fixture.marker_mount_calibration_deg,
                                  "target_center_u_px": camera.target_u_px,
                                  "final_yaw_trim_limit_radps": Dock.FINAL_YAW_TRIM_LIMIT_RADPS
                                  if fixture.trim_dock_yaw else None,
                                  "mechanical_dock_reference": asdict(dock_reference)}
    return {"rows": rows, "gates": gates, "events": events, "frames": camera_stream.frames,
            "route_origins": [pose.tolist() for pose in route.origins] if route else [],
            "scheduled_duration_s": sum(route.durations) if route else None,
            "initial_solver_state": asdict(start), "initial_axle_pose": initial_pose.tolist(),
            "configuration": config, "completed_s": completed_s, "termination_reason": reason,
            "stop_phase_start_s": stop_phase_start, "image_replay": image_replay}
