"""OKUMA 12 — Çözücü durumu ile grafik/CSV aynı koordinat olmayabilir.

B (gövde COM) entegre edilir; A (tahrik aksı orta noktası) CSV'ye yazılır.
Sütun isimleri MATLAB referansıyla aynıdır. Enerji ve momentum ayrıca tutulur.
"""
import numpy as np
from ..physics.geometry import geometry_at


def result_record(time_s, state, torques, parameters, evaluation, initial_energy_J):
    g, d = parameters.geometry, parameters.dynamics
    geometry = geometry_at(state, parameters)
    com = np.array([state.body_x_m, state.body_y_m])
    axle = com-geometry.body_rotation @ np.array(d.body_com_m[:2])
    velocity = np.array([state.body_velocity_x_mps, state.body_velocity_y_mps])
    body_velocity = geometry.body_rotation.T @ velocity
    caster_center = com+geometry.caster_from_com_world_m
    caster_velocity = geometry.caster_velocity_jacobian @ state.velocities()
    momentum = d.body_lumped_mass_kg*velocity+d.caster_lumped_mass_kg*caster_velocity
    angular_momentum = (
        d.body_lumped_mass_kg*(com[0]*velocity[1]-com[1]*velocity[0])
        + d.body_yaw_inertia_kgm2*state.body_yaw_rate_radps
        + d.caster_lumped_mass_kg*(caster_center[0]*caster_velocity[1]-caster_center[1]*caster_velocity[0])
        + d.caster_swivel_inertia_com_kgm2*state.caster_world_rate_radps
    )
    contacts, slips = evaluation.contacts, evaluation.slips
    row = {
        "time_s": time_s, "left_drive_torque_Nm": torques.left_Nm, "right_drive_torque_Nm": torques.right_Nm,
        "x_m": axle[0], "y_m": axle[1], "yaw_rad": state.body_yaw_rad,
        "caster_angle_body_rad": state.caster_angle_relative_rad,
        "caster_angle_world_rad": state.caster_world_angle_rad,
        "body_com_x_world_m": state.body_x_m, "body_com_y_world_m": state.body_y_m,
        "body_long_velocity_mps": body_velocity[0], "body_lat_velocity_mps": body_velocity[1],
        "body_yaw_rate_radps": state.body_yaw_rate_radps,
        "caster_relative_rate_radps": state.caster_relative_rate_radps,
        "left_wheel_rate_radps": state.left_wheel_rate_radps, "right_wheel_rate_radps": state.right_wheel_rate_radps,
        "caster_roll_rate_radps": state.caster_roll_rate_radps,
        "left_fx_world_N": contacts.left_force_world_N[0], "left_fy_world_N": contacts.left_force_world_N[1],
        "right_fx_world_N": contacts.right_force_world_N[0], "right_fy_world_N": contacts.right_force_world_N[1],
        "caster_fx_world_N": contacts.caster_force_world_N[0], "caster_fy_world_N": contacts.caster_force_world_N[1],
        "normal_left_N": evaluation.normal_loads_N[0], "normal_right_N": evaluation.normal_loads_N[1],
        "normal_caster_N": evaluation.normal_loads_N[2],
        "left_long_slip_mps": slips.left_mps[0], "left_lat_slip_mps": slips.left_mps[1],
        "right_long_slip_mps": slips.right_mps[0], "right_lat_slip_mps": slips.right_mps[1],
        "caster_long_slip_mps": slips.caster_mps[0], "caster_lat_slip_mps": slips.caster_mps[1],
        "swivel_bearing_torque_Nm": contacts.bearing_torque_Nm, "bore_ground_torque_Nm": contacts.bore_torque_Nm,
        "caster_external_yaw_moment_Nm": evaluation.caster_yaw_moment_Nm,
        "kinetic_energy_J": evaluation.kinetic_energy_J, "motor_power_W": evaluation.motor_power_W,
        "dissipation_power_W": evaluation.dissipation_power_W,
        "motor_work_J": state.motor_work_J, "dissipated_energy_J": state.dissipated_energy_J,
        "energy_balance_error_J": evaluation.kinetic_energy_J-initial_energy_J+state.dissipated_energy_J-state.motor_work_J,
        "linear_momentum_x_kgmps": momentum[0], "linear_momentum_y_kgmps": momentum[1],
        "angular_momentum_z_kgm2ps": angular_momentum,
    }
    return {name: float(value) for name, value in row.items()}
