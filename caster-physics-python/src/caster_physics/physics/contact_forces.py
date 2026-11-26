"""OKUMA 6 — Kaymayı hangi kuvvetler ve direnç momentleri karşılar?

Kuvvet kaymaya zıttır, |F|≤mu*N. Caster kuvveti ve zemin dönme (bore)
momenti aynı sürtünme bütçesini paylaşır. Yasa sentetik/yumuşaktır;
tam statik kilitlenme, kauçuk teması veya shimmy çözümü değildir.
"""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class ContactForces:
    left_force_world_N: np.ndarray
    right_force_world_N: np.ndarray
    caster_force_world_N: np.ndarray
    left_force_local_N: np.ndarray
    right_force_local_N: np.ndarray
    caster_force_local_N: np.ndarray
    left_rolling_torque_Nm: float
    right_rolling_torque_Nm: float
    caster_rolling_torque_Nm: float
    bearing_torque_Nm: float
    bore_torque_Nm: float


def passive_friction(slip, longitudinal_damping, lateral_damping, limit_N):
    """F = -z*L/sqrt(L²+z·z), z=[kx*s_long, ky*s_lat]."""
    raw = np.array([longitudinal_damping*slip[0], lateral_damping*slip[1]])
    if limit_N <= 0:
        return np.zeros(2)
    return -raw*limit_N/np.sqrt(limit_N**2+raw @ raw)


def contact_forces(state, parameters, geometry, slips, normal_loads_N, caster_contact_enabled=True):
    g, d = parameters.geometry, parameters.dynamics
    left_normal, right_normal, caster_normal = normal_loads_N
    left_local = passive_friction(slips.left_mps, d.drive_long_slip_damping_Nspm,
                                 d.drive_lat_slip_damping_Nspm, d.drive_friction_coefficient*left_normal)
    right_local = passive_friction(slips.right_mps, d.drive_long_slip_damping_Nspm,
                                  d.drive_lat_slip_damping_Nspm, d.drive_friction_coefficient*right_normal)
    # Bore momenti mutlak caster dönme hızına karşı koyar (thetaDot+phiDot).
    limit = d.caster_friction_coefficient*caster_normal
    spin = limit*state.caster_world_rate_radps/d.bore_regularization_radps if d.caster_bore_effective_radius_m > 0 else 0
    raw = np.array([d.caster_long_slip_damping_Nspm*slips.caster_mps[0],
                    d.caster_lat_slip_damping_Nspm*slips.caster_mps[1], spin])
    scale = limit/np.sqrt(limit**2+raw @ raw) if limit > 0 else 0.0
    caster_local = -scale*raw[:2]
    bore = scale*raw[2]*d.caster_bore_effective_radius_m

    def rolling_resistance(normal, radius, rate, coefficient, damping):
        return coefficient*normal*radius*np.tanh(rate/d.rolling_regularization_radps)+damping*rate

    left_rolling = rolling_resistance(left_normal, g.drive_radius_m, state.left_wheel_rate_radps,
                                      d.drive_rolling_resistance_coefficient, d.drive_roll_viscous_NmspRad)
    right_rolling = rolling_resistance(right_normal, g.drive_radius_m, state.right_wheel_rate_radps,
                                       d.drive_rolling_resistance_coefficient, d.drive_roll_viscous_NmspRad)
    caster_rolling = rolling_resistance(caster_normal, g.caster_radius_m, state.caster_roll_rate_radps,
                                        d.caster_rolling_resistance_coefficient, d.caster_roll_viscous_NmspRad)
    # Bearing iki cisim arasındadır; göreli phiDot ile enerji tüketir.
    bearing = (d.swivel_bearing_viscous_NmspRad*state.caster_relative_rate_radps
               + d.swivel_bearing_coulomb_Nm*np.tanh(state.caster_relative_rate_radps/d.swivel_regularization_radps))
    if not caster_contact_enabled:
        caster_local, caster_rolling, bore = np.zeros(2), 0.0, 0.0
    drive_frame = np.column_stack((geometry.drive_forward, geometry.drive_lateral))
    caster_frame = np.column_stack((geometry.caster_forward, geometry.caster_lateral))
    return ContactForces(drive_frame @ left_local, drive_frame @ right_local, caster_frame @ caster_local,
                         left_local, right_local, caster_local, left_rolling, right_rolling,
                         caster_rolling, bearing, bore)
