"""V2 experiment layer. The preserved V1 remains the only M2 physics kernel."""
from dataclasses import asdict, dataclass, replace
from pathlib import Path
import hashlib
import json
import sys

import numpy as np

ROOT = Path(__file__).resolve().parent
V1 = ROOT.parent / "caster-physics-python"
sys.path.insert(0, str(V1 / "src"))

from caster_physics.parameters import load_parameters as load_v1_parameters, validate_parameters
from caster_physics.state import State, Torques
from caster_physics.physics.geometry import geometry_at
from caster_physics.physics.motion import evaluate_motion
from caster_physics.physics.energy import kinetic_energy
from caster_physics.simulation.rk4 import rk4_step
from caster_physics.simulation.records import result_record


def load_parameters(directory=None):
    """V2 working parameters; the frozen V1/MATLAB inputs remain unchanged."""
    return load_v1_parameters(ROOT / 'parameters' if directory is None else directory)


def verify_v1():
    hashes = json.loads((ROOT / "evidence/v1_sha256.json").read_text())
    changed = [name for name, digest in hashes.items()
               if not (V1 / name).is_file()
               or hashlib.sha256((V1 / name).read_bytes()).hexdigest() != digest]
    if changed:
        raise ValueError(f"Preserved V1 changed: {changed}")
    return {"passed": True, "checked_files": len(hashes)}


def wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


@dataclass(frozen=True)
class Segment:
    distance_m: float
    yaw_change_rad: float
    duration_s: float = 2.0


MANEUVERS = {
    "straight": (Segment(.30, 0),),
    "arc_turn": (Segment(.30, np.pi / 3),),
    "reversal": (Segment(.30, 0), Segment(-.30, 0)),
    "s_turn": (Segment(.30, np.pi / 3), Segment(.30, -np.pi / 3)),
}
PROFILES = ("baseline", "smooth")
SURFACES = {
    "reference_synthetic": {},
    "high_drag_synthetic": {
        "drive_friction_coefficient": .45, "caster_friction_coefficient": .45,
        "drive_rolling_resistance_coefficient": .016,
        "caster_rolling_resistance_coefficient": .024,
        "caster_long_slip_damping_Nspm": 140., "caster_lat_slip_damping_Nspm": 175.,
    },
}
DESIGNS = {
    "nominal": {},
    "trail_30mm": {"caster_trail_m": .030},
    "trail_70mm": {"caster_trail_m": .070},
    "radius_30mm": {"caster_radius_m": .030},
    "radius_50mm": {"caster_radius_m": .050},
    "swivel_damping_half": {"swivel_bearing_viscous_NmspRad": .002},
    "swivel_damping_double": {"swivel_bearing_viscous_NmspRad": .008},
}


def case_parameters(surface, design="nominal"):
    base = load_parameters()
    changes = DESIGNS[design]
    geometry = {key: value for key, value in changes.items() if hasattr(base.geometry, key)}
    dynamics = {key: value for key, value in changes.items() if hasattr(base.dynamics, key)}
    if 'caster_radius_m' in geometry:
        # Same wheel mass and uniform-cylinder assumption, with the new radius.
        dynamics['caster_roll_inertia_kgm2'] = base.dynamics.caster_roll_inertia_kgm2*(geometry['caster_radius_m']/base.geometry.caster_radius_m)**2
    parameters = replace(base, geometry=replace(base.geometry, **geometry),
                         dynamics=replace(base.dynamics, **SURFACES[surface], **dynamics))
    validate_parameters(parameters)
    return parameters


def advance_pose(pose, distance, yaw_change):
    """Exact SE(2) constant-curvature displacement, including a pure rotation."""
    x, y, yaw = pose
    if abs(yaw_change) < 1e-12:
        return np.array([x + distance * np.cos(yaw), y + distance * np.sin(yaw), yaw])
    radius = distance / yaw_change
    return np.array([x + radius * (np.sin(yaw + yaw_change) - np.sin(yaw)),
                     y - radius * (np.cos(yaw + yaw_change) - np.cos(yaw)), yaw + yaw_change])


def reference_at(time_s, maneuver, profile):
    if maneuver not in MANEUVERS or profile not in PROFILES or not np.isfinite(time_s) or time_s < 0:
        raise ValueError("Unknown maneuver/profile or invalid reference time")
    pose, remaining = np.zeros(3), time_s
    for segment in MANEUVERS[maneuver]:
        # Quintic progress peaks at 1.875; stretch time to match baseline peak speed.
        duration = segment.duration_s * (1.875 if profile == "smooth" else 1.)
        if remaining < duration:
            u = remaining / duration
            progress = 10*u**3 - 15*u**4 + 6*u**5 if profile == "smooth" else u
            rate = (30*u**2 - 60*u**3 + 30*u**4) / duration if profile == "smooth" else 1/duration
            return (advance_pose(pose, segment.distance_m * progress, segment.yaw_change_rad * progress),
                    segment.distance_m * rate, segment.yaw_change_rad * rate)
        pose = advance_pose(pose, segment.distance_m, segment.yaw_change_rad)
        remaining -= duration
    return pose, 0., 0.


def profile_duration(maneuver, profile):
    return sum(segment.duration_s for segment in MANEUVERS[maneuver]) * (1.875 if profile == "smooth" else 1.)


def wheel_odometry(times, left_rates, right_rates, parameters, initial_pose):
    times, left_rates, right_rates = [np.asarray(values, dtype=float) for values in (times, left_rates, right_rates)]
    if (times.ndim != 1 or len(times) < 2 or left_rates.shape != times.shape or right_rates.shape != times.shape
            or not np.all(np.isfinite([times, left_rates, right_rates])) or np.any(np.diff(times) <= 0)):
        raise ValueError("Odometry requires aligned finite samples and strictly increasing times")
    pose = np.asarray(initial_pose, dtype=float)
    if pose.shape != (3,) or not np.all(np.isfinite(pose)):
        raise ValueError("Initial axle pose must contain three finite values")
    poses = [pose.copy()]
    for i, dt in enumerate(np.diff(times)):
        left = (left_rates[i] + left_rates[i+1]) / 2
        right = (right_rates[i] + right_rates[i+1]) / 2
        distance = parameters.geometry.drive_radius_m * (left + right) * dt / 2
        yaw = parameters.geometry.drive_radius_m * (right - left) * dt / parameters.geometry.drive_track_m
        pose = advance_pose(pose, distance, yaw)
        poses.append(pose)
    return np.asarray(poses)


def controller_gains(parameters):
    """Selected design requirements, not experimentally identified coefficients."""
    straight = MANEUVERS['straight'][0]
    design_speed = abs(straight.distance_m)/straight.duration_s
    lateral_natural_frequency = 1.0
    wheel_poles = (.8, .5)
    J, h = parameters.dynamics.drive_roll_inertia_kgm2, parameters.dynamics.command_sample_dt_s
    return {'tracking_x': 1/.5,
            'tracking_y': lateral_natural_frequency**2/design_speed**2,
            'tracking_heading': 2*lateral_natural_frequency,
            'wheel_p': J*(1-np.prod(wheel_poles))/h,
            'wheel_i': J*np.prod([1-pole for pole in wheel_poles])/h**2,
            'terminal_rho': 1/.5, 'terminal_alpha': 1/(1/3), 'terminal_heading': 1/(1/3)}


class TrackingController:
    """Synthetic tracking, terminal position/yaw recovery, and bounded wheel PI."""

    def __init__(self, parameters, maneuver, profile):
        self.parameters, self.maneuver, self.profile = parameters, maneuver, profile
        self.gains = controller_gains(parameters)
        self.integral_torque_Nm = np.zeros(2)
        self.last_time_s = None
        self.phase = "tracking"
        self.terminal_direction = None

    def wheel_torques(self, state, target_rates, dt):
        error = np.asarray(target_rates) - [state.left_wheel_rate_radps, state.right_wheel_rate_radps]
        candidate = self.integral_torque_Nm + self.gains['wheel_i']*dt*error
        raw = self.gains['wheel_p']*error + candidate
        clipped = np.clip(raw, -.5, .5)
        # Integrate only when unsaturated or when the error unwinds saturation.
        allowed = (abs(raw) <= .5) | (error*(raw-clipped) <= 0)
        self.integral_torque_Nm = np.where(allowed, candidate, self.integral_torque_Nm)
        return Torques(*np.clip(self.gains['wheel_p']*error + self.integral_torque_Nm, -.5, .5))

    def __call__(self, time_s, state):
        g, d = self.parameters.geometry, self.parameters.dynamics
        target, speed, yaw_rate = reference_at(time_s, self.maneuver, self.profile)
        rotation = geometry_at(state, self.parameters).body_rotation
        axle = np.array([state.body_x_m, state.body_y_m]) - rotation @ np.array(d.body_com_m[:2])
        delta = target[:2] - axle
        ex, ey = rotation.T @ delta
        yaw_error = wrap(target[2] - state.body_yaw_rad)
        if time_s < profile_duration(self.maneuver, self.profile):
            commanded_speed = speed*np.cos(yaw_error) + self.gains['tracking_x']*ex
            commanded_yaw = yaw_rate + self.gains['tracking_y']*speed*ey + self.gains['tracking_heading']*np.sin(yaw_error)
        else:
            distance = np.linalg.norm(delta)
            if self.phase == "tracking" or (self.phase == "align" and distance > .012):
                self.phase, self.terminal_direction = "position", None
            if distance <= .006:
                self.phase = "align"
            if self.phase == "position":
                bearing = np.arctan2(delta[1], delta[0])
                if self.terminal_direction is None:
                    self.terminal_direction = 1. if np.cos(bearing-state.body_yaw_rad) >= 0 else -1.
                effective_yaw = state.body_yaw_rad + (np.pi if self.terminal_direction < 0 else 0.)
                alpha = wrap(bearing-effective_yaw)
                commanded_speed = self.terminal_direction*self.gains['terminal_rho']*distance*np.cos(alpha)
                commanded_yaw = self.gains['terminal_alpha']*alpha + self.gains['terminal_rho']*np.sin(alpha)*np.cos(alpha)
            else:
                commanded_speed, commanded_yaw = 0., self.gains['terminal_heading']*np.sin(yaw_error)
        commanded_speed = np.clip(commanded_speed, -.25, .25)
        commanded_yaw = np.clip(commanded_yaw, -1., 1.)
        self.commanded_speed_mps = float(commanded_speed)
        self.commanded_yaw_radps = float(commanded_yaw)
        target_rates = np.array([commanded_speed-g.drive_track_m*commanded_yaw/2,
                                 commanded_speed+g.drive_track_m*commanded_yaw/2]) / g.drive_radius_m
        dt = d.command_sample_dt_s if self.last_time_s is None else time_s-self.last_time_s
        if not np.isfinite(dt) or dt < 0:
            raise ValueError("Controller calls must have finite nondecreasing timestamps")
        self.last_time_s = time_s
        return self.wheel_torques(state, target_rates, dt)


def integrate(parameters, duration_s, start, torque_at, reference=None):
    """Sample controller once per command period, hold torque through all RK4 stages."""
    d = parameters.dynamics
    ratios = np.array([duration_s, d.command_sample_dt_s, d.result_sample_dt_s]) / d.solver_step_s
    if not np.all(np.isfinite(ratios)) or np.any(ratios < 1) or np.any(abs(ratios - np.rint(ratios)) > 1e-8):
        raise ValueError("Duration and sampling intervals must be positive integer solver steps")
    steps, per_command, per_record = np.rint(ratios).astype(int)
    vector, records = start.as_vector(), []
    initial_energy = kinetic_energy(start, parameters)
    for index in range(steps + 1):
        time_s = index*d.solver_step_s
        state = State.from_vector(vector)
        if index % per_command == 0:
            torques = torque_at(time_s, state)
        if index % per_record == 0 or index == steps:
            evaluation = evaluate_motion(state, torques, parameters)
            record = result_record(time_s, state, torques, parameters, evaluation, initial_energy)
            if reference is not None:
                pose, speed, yaw_rate = reference(time_s)
                record.update(ref_x_m=float(pose[0]), ref_y_m=float(pose[1]), ref_yaw_rad=float(pose[2]),
                              command_speed_mps=float(speed), command_yaw_rate_radps=float(yaw_rate))
            if isinstance(torque_at, TrackingController):
                record.update(controller_speed_mps=torque_at.commanded_speed_mps,
                              controller_yaw_rate_radps=torque_at.commanded_yaw_radps,
                              controller_phase_code=float({"tracking": 0, "position": 1, "align": 2}[torque_at.phase]),
                              left_integral_torque_Nm=float(torque_at.integral_torque_Nm[0]),
                              right_integral_torque_Nm=float(torque_at.integral_torque_Nm[1]))
            records.append(record)
        if index == steps:
            break
        vector = rk4_step(vector, d.solver_step_s, lambda values:
                          evaluate_motion(State.from_vector(values), torques, parameters).derivative)
    return records


def pose_errors(predicted, reference):
    predicted, reference = np.asarray(predicted), np.asarray(reference)
    if predicted.shape != reference.shape or predicted.ndim != 2 or predicted.shape[1] != 3 or len(predicted) < 2:
        raise ValueError("Pose comparison requires equal Nx3 arrays, N >= 2")
    if not np.all(np.isfinite([predicted, reference])):
        raise ValueError("Nonfinite pose comparison")
    position = np.linalg.norm(predicted[:, :2] - reference[:, :2], axis=1)
    yaw = wrap(predicted[:, 2] - reference[:, 2])
    return {"position_rmse_m": float(np.sqrt(np.mean(position**2))),
            "position_max_m": float(position.max()), "endpoint_error_m": float(position[-1]),
            "yaw_rmse_rad": float(np.sqrt(np.mean(yaw**2)))}


def completion_time(records, target, profile_end_s):
    """Report the start of the first verified 0.25s dwell, using physical timestamps."""
    started = None
    for row in records:
        valid = (row["time_s"] >= profile_end_s
                 and np.hypot(row["x_m"]-target[0], row["y_m"]-target[1]) <= .02
                 and abs(wrap(row["yaw_rad"]-target[2])) <= .05
                 and np.hypot(row["body_long_velocity_mps"], row["body_lat_velocity_mps"]) <= .02
                 and abs(row["body_yaw_rate_radps"]) <= .05)
        if not valid:
            started = None
        elif started is None:
            started = row["time_s"]
        if started is not None and row["time_s"]-started >= .25-1e-12:
            return float(started)
    return None


def summarize(records, parameters, maneuver, profile):
    def column(name):
        return np.array([row[name] for row in records])
    pose = np.column_stack([column(name) for name in ("x_m", "y_m", "yaw_rad")])
    target = np.column_stack([column(name) for name in ("ref_x_m", "ref_y_m", "ref_yaw_rad")])
    times = column("time_s")
    odometry = wheel_odometry(times, column("left_wheel_rate_radps"), column("right_wheel_rate_radps"), parameters, pose[0])
    for row, ideal in zip(records, odometry):
        row.update(m0_encoder_x_m=float(ideal[0]), m0_encoder_y_m=float(ideal[1]), m0_encoder_yaw_rad=float(ideal[2]))
    # Spatial path error is separate from synchronized tracking error and profile duration.
    path = np.array([reference_at(t, maneuver, "baseline")[0][:2]
                     for t in np.linspace(0, profile_duration(maneuver, "baseline"), 501)])
    path_error = np.array([np.min(np.linalg.norm(path - point, axis=1)) for point in pose[:, :2]])
    acceleration = np.gradient(column("body_long_velocity_mps"), times)
    jerk = np.gradient(acceleration, times)
    profile_end = profile_duration(maneuver, profile)
    completion = completion_time(records, target[-1], profile_end)
    legacy_window = [row for row in records if row["time_s"] <= profile_end+1.+1e-12]
    return {
        "tracking": pose_errors(pose, target),
        "m0_encoder_vs_synthetic_m2": pose_errors(odometry, pose),
        "path_rmse_m": float(np.sqrt(np.mean(path_error**2))),
        "scheduled_duration_s": profile_duration(maneuver, profile),
        "observed_duration_s": float(times[-1]), "completion_time_s": completion,
        "completion_within_original_1s_window_s": completion_time(legacy_window, target[-1], profile_end),
        "completion_rule": "after profile end: 2cm, .05rad, planar speed .02m/s, .05rad/s held .25s by timestamps",
        "tracking_during_profile": pose_errors(pose[times <= profile_end], target[times <= profile_end]),
        "tracking_on_original_window": pose_errors(pose[times <= profile_end+1.+1e-12], target[times <= profile_end+1.+1e-12]),
        "jerk_rms_during_profile_mps3": float(np.sqrt(np.mean(jerk[times <= profile_end]**2))),
        "final_position_error_m": float(np.linalg.norm(pose[-1, :2]-target[-1, :2])),
        "final_yaw_error_rad": float(abs(wrap(pose[-1, 2]-target[-1, 2]))),
        "jerk_rms_mps3": float(np.sqrt(np.mean(jerk**2))),
        "motor_work_signed_J": float(column("motor_work_J")[-1]),
        "dissipated_energy_J": float(column("dissipated_energy_J")[-1]),
        "caster_lateral_slip_rms_mps": float(np.sqrt(np.mean(column("caster_lat_slip_mps")**2))),
        "caster_yaw_moment_peak_Nm": float(np.max(abs(column("caster_external_yaw_moment_Nm")))),
        "energy_balance_max_J": float(np.max(abs(column("energy_balance_error_J")))),
        "dissipation_min_W": float(column("dissipation_power_W").min()),
        "torque_peak_Nm": float(max(abs(column("left_drive_torque_Nm")).max(), abs(column("right_drive_torque_Nm")).max())),
    }


def run_case(maneuver, profile, surface, design="nominal", step_scale=1.):
    parameters = case_parameters(surface, design)
    parameters = replace(parameters, dynamics=replace(parameters.dynamics,
                         solver_step_s=parameters.dynamics.solver_step_s*step_scale))
    validate_parameters(parameters)
    start = replace(State.at_rest(parameters), caster_angle_relative_rad=np.pi/4)
    reference = lambda time: reference_at(time, maneuver, profile)
    controller = TrackingController(parameters, maneuver, profile)
    records = integrate(parameters, profile_duration(maneuver, profile) + 8., start, controller, reference)
    return records, summarize(records, parameters, maneuver, profile), asdict(parameters)
