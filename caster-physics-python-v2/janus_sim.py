"""Run the V2.3 RK4 plant with JANUS task controllers and time-aligned logging."""
from collections import deque
from dataclasses import asdict, replace
from math import cos, hypot, sin

import numpy as np

from qr_camera import Camera, DOCK_ID, Frame, Marker, estimate_marker, mechanical_errors, project
from qr_docking import Dock, FrameGate
from scenarios import Dwell, Route, axle_state, terminal_ok
from twin import (State, Torques, TrackingController, case_parameters, evaluate_motion,
                  kinetic_energy, result_record, rk4_step, validate_parameters, wrap)


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
    return replace(State.at_rest(p), body_x_m=cos(yaw)*cx-sin(yaw)*cy,
                   body_y_m=fixture.y_m+sin(yaw)*cx+cos(yaw)*cy, body_yaw_rad=yaw,
                   caster_angle_relative_rad=np.deg2rad(fixture.caster_deg))


class CameraStream:
    def __init__(self, camera, marker, seed, noisy):
        self.camera, self.marker = camera, marker
        self.rng = np.random.default_rng(seed) if noisy else None
        self.queue = deque()
        self.frames = []
        self.last = None

    def sample(self, index, time_s, dt, pose, mode):
        frame_ticks = round(self.camera.frame_period_s/dt)
        if index % frame_ticks == 0 and mode != "stream_cut":
            delay = (.07 + .02*(index//frame_ticks % 2)) if self.rng is not None else 0.
            if mode == "stale":
                delay = .27
            if mode == "duplicate" and self.last is not None:
                frame = replace(self.last, arrival_time_s=time_s)
            else:
                identity = "JANUS-OTHER-99" if mode == "wrong" else DOCK_ID
                candidates = () if mode == "loss" else (project(pose, self.camera, self.marker, identity, self.rng),)
                frame = Frame(time_s, round(time_s+delay, 9), candidates)
            self.queue.append(frame)
            self.last = frame
        arrivals = []
        while self.queue and self.queue[0].arrival_time_s <= time_s+1e-10:
            frame = self.queue.popleft()
            arrivals.append(frame)
            self.frames.append(frame)
        return arrivals


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
    use_camera = fixture.case.startswith("Q") or fixture.case == "O07"
    event_types = {event for _, event in fixture.events}
    camera = Camera(left_m=.035, yaw_rad=np.deg2rad(4), target_u_px=510.*np.tan(np.arctan2(.035, .22)-np.deg2rad(4))+320.) if "camera_offset" in event_types else Camera()
    marker = Marker()
    camera_stream = CameraStream(camera, marker, fixture.seed, "delay_noise" in event_types)
    frame_gate = FrameGate(camera)
    observation = frame_gate.current
    dock = Dock(camera, marker.side_m, fixture.qr_yaw_gain, fixture.search_sweep) if use_camera else None
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
    for tick in range(round(fixture.duration_s/dt)+1):
        t = round(tick*dt, 9)
        state = State.from_vector(vector)
        pose, v, lateral = axle_state(state, p)
        stopped = terminal_ok((0., 0., hypot(v, lateral), abs(state.body_yaw_rate_radps)))
        while schedule and schedule[0][0] <= t+1e-10:
            _, event = schedule.pop(0)
            transition_events.append(dict(time_s=t, event=event, reason="fixture"))
            if event in {"visible", "wrong", "loss", "stale", "duplicate", "stream_cut"}:
                mode = event
            elif event in {"obstacle", "command_timeout", "operator_cancel",
                           "odometry_timeout", "odometry_clock_regression"}:
                interlock = event
            elif event == "pedestrian":
                pedestrian_hold = True
            elif event == "authorized_resume":
                pedestrian_hold = False
            elif event == "low_soc":
                transit = False
            elif event == "brake_step":
                braking = True
        if high_drag is not None and step_parameters is p and pose[0] >= .30:
            step_parameters = high_drag
            transition_events.append(dict(time_s=t, event="surface_change", reason="axle_crossed_boundary"))
        estimator_range, estimator_heading = None, None
        current_bms_s, mechanical = None, (None, None)
        if use_camera:
            arrivals = camera_stream.sample(tick, t, dt, pose, mode)
            observation = frame_gate.update(t, arrivals)
            mechanical = mechanical_errors(pose, marker, Dock.STANDOFF_AXLE_M)
            if observation.valid:
                estimator_range, estimator_heading = estimate_marker(observation.candidate, camera, marker.side_m)
            route_goal = np.array([marker.x_m-Dock.STANDOFF_AXLE_M, marker.y_m, marker.yaw_rad])
        if interlock and not use_camera:
            requested_v, requested_w, phase, why = 0., 0., "abort", interlock
        elif route:
            requested_v, requested_w, route_goal, phase = route.update(t, pose, v, lateral, state.body_yaw_rate_radps)
            why = phase
        elif use_camera and not transit:
            if ("virtual_charge" in event_types and dock.wait_s is not None and last_bms_s is None
                    and t-dock.wait_s >= .30-1e-10):
                last_bms_s = current_bms_s = t
                transition_events.append(dict(time_s=t, event="virtual_bms_frame", reason="synthetic_charge_source"))
            requested_v, requested_w, why = dock.update(t, observation, stopped, mechanical, interlock, current_bms_s)
            phase = dock.phase
        elif fixture.case == "M01":
            requested_v, requested_w, phase = 0., 0., "idle"
            why = "zero_torque_fixture"
        elif fixture.case == "O03" and pedestrian_hold:
            requested_v, requested_w, phase, why = 0., 0., "hold", "pedestrian_interlock_latched"
        elif fixture.case == "O03" and (pose[0] >= .54 or braking):
            braking = True
            requested_v, requested_w, phase, why = 0., 0., "stop", "transit_target"
        elif braking:
            requested_v, requested_w, phase, why = 0., 0., "stop", "brake_step"
        else:
            requested_v, requested_w, phase, why = .14, 0., "transit", "fixture_velocity_request"
        limited_v = float(np.clip(requested_v, -.25, .25))
        limited_w = float(np.clip(requested_w, -1., 1.))
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
        rows.append(row)
        finished = (route and route.done) or (dock and dock.done) or phase in {"abort", "stop"} and not use_camera
        if finished:
            if stop_phase_start is None:
                stop_phase_start = t
            if motion_stop.update(t, stopped):
                completed_s = t
                reason = "route_gates_accepted" if route else why
                break
        if tick == round(fixture.duration_s/dt):
            if fixture.case == "M01":
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
              "target_axle_standoff_m": Dock.STANDOFF_AXLE_M if use_camera else None}
    return {"rows": rows, "gates": gates, "events": events, "frames": camera_stream.frames,
            "route_origins": [pose.tolist() for pose in route.origins] if route else [],
            "scheduled_duration_s": sum(route.durations) if route else None,
            "initial_solver_state": asdict(start), "initial_axle_pose": initial_pose.tolist(),
            "configuration": config, "completed_s": completed_s, "termination_reason": reason,
            "stop_phase_start_s": stop_phase_start}
