"""OKUMA 9 — Bütün fizik zincirini burada sırayla oku.

Geometri -> destek -> kayma -> temas -> M,bias,Q -> ivmeler -> 13 türev.
Temel denklem M @ ivme + bias = Q; ters matris alınmaz, sistem çözülür.
"""
from dataclasses import dataclass
import numpy as np

from ..state import VelocityIndex as V, derivative_vector
from .geometry import geometry_at
from .support_loads import support_loads
from .wheel_slip import wheel_slips, WheelSlips
from .contact_forces import contact_forces, ContactForces
from .inertia import mass_matrix_and_bias
from .energy import kinetic_energy, power_balance


@dataclass(frozen=True)
class MotionEvaluation:
    derivative: np.ndarray
    contacts: ContactForces
    slips: WheelSlips
    normal_loads_N: np.ndarray
    kinetic_energy_J: float
    motor_power_W: float
    dissipation_power_W: float
    caster_yaw_moment_Nm: float


def generalized_forces(torques, parameters, geometry, contacts):
    """Aynı temas kuvveti hem gövdeyi hem swivel/roll hareketini etkiler."""
    g = parameters.geometry
    left_map = geometry.left_velocity_jacobian.copy()
    right_map = geometry.right_velocity_jacobian.copy()
    caster_map = geometry.caster_velocity_jacobian.copy()
    left_map[:, V.LEFT_WHEEL_ROLL] = -g.drive_radius_m*geometry.drive_forward
    right_map[:, V.RIGHT_WHEEL_ROLL] = -g.drive_radius_m*geometry.drive_forward
    caster_map[:, V.CASTER_WHEEL_ROLL] = -g.caster_radius_m*geometry.caster_forward
    force = (left_map.T @ contacts.left_force_world_N
             + right_map.T @ contacts.right_force_world_N
             + caster_map.T @ contacts.caster_force_world_N)
    force[V.LEFT_WHEEL_ROLL] += torques.left_Nm-contacts.left_rolling_torque_Nm
    force[V.RIGHT_WHEEL_ROLL] += torques.right_Nm-contacts.right_rolling_torque_Nm
    force[V.CASTER_WHEEL_ROLL] -= contacts.caster_rolling_torque_Nm
    force[V.CASTER_RELATIVE_SWIVEL] -= contacts.bearing_torque_Nm
    # Zemin bore momenti dış momenttir; theta ve phi denklemlerine birlikte girer.
    force[[V.BODY_YAW, V.CASTER_RELATIVE_SWIVEL]] -= contacts.bore_torque_Nm
    return force


def evaluate_motion(state, torques, parameters, caster_contact_enabled=True):
    geometry = geometry_at(state, parameters)
    normal_loads = support_loads(state, parameters)
    slips = wheel_slips(state, parameters, geometry)
    contacts = contact_forces(state, parameters, geometry, slips, normal_loads, caster_contact_enabled)
    matrix, bias = mass_matrix_and_bias(state, parameters, geometry)
    force = generalized_forces(torques, parameters, geometry, contacts)
    accelerations = np.linalg.solve(matrix, force-bias)
    motor_power, dissipation_power = power_balance(state, torques, contacts, slips)
    derivative = derivative_vector(state, accelerations, motor_power, dissipation_power)
    offset = geometry.caster_from_com_world_m
    caster_force = contacts.caster_force_world_N
    caster_moment = offset[0]*caster_force[1]-offset[1]*caster_force[0]-contacts.bore_torque_Nm
    return MotionEvaluation(derivative, contacts, slips, normal_loads,
                            kinetic_energy(state, parameters, geometry), motor_power, dissipation_power, caster_moment)
