"""OKUMA 8 — Enerji hareketi sürmez; bağımsız hesapla dengeyi denetleriz.

T-T0 + D-W = 0. Motor işi W, kayıp D, kinetik enerji T [J].
"""
import numpy as np
from .geometry import geometry_at


def kinetic_energy(state, parameters, geometry=None):
    d = parameters.dynamics
    geometry = geometry if geometry is not None else geometry_at(state, parameters)
    caster_velocity = geometry.caster_velocity_jacobian @ state.velocities()
    return (
        .5*d.body_lumped_mass_kg*(state.body_velocity_x_mps**2+state.body_velocity_y_mps**2)
        + .5*d.caster_lumped_mass_kg*(caster_velocity @ caster_velocity)
        + .5*d.body_yaw_inertia_kgm2*state.body_yaw_rate_radps**2
        + .5*d.caster_swivel_inertia_com_kgm2*state.caster_world_rate_radps**2
        + .5*d.drive_roll_inertia_kgm2*(state.left_wheel_rate_radps**2+state.right_wheel_rate_radps**2)
        + .5*d.caster_roll_inertia_kgm2*state.caster_roll_rate_radps**2
    )


def power_balance(state, torques, contacts, slips):
    motor_power = torques.left_Nm*state.left_wheel_rate_radps+torques.right_Nm*state.right_wheel_rate_radps
    dissipation = (
        -contacts.left_force_local_N @ slips.left_mps
        -contacts.right_force_local_N @ slips.right_mps
        -contacts.caster_force_local_N @ slips.caster_mps
        + contacts.left_rolling_torque_Nm*state.left_wheel_rate_radps
        + contacts.right_rolling_torque_Nm*state.right_wheel_rate_radps
        + contacts.caster_rolling_torque_Nm*state.caster_roll_rate_radps
        + contacts.bearing_torque_Nm*state.caster_relative_rate_radps
        + contacts.bore_torque_Nm*state.caster_world_rate_radps
    )
    return motor_power, dissipation
