"""OKUMA 7 — Caster ile gövdenin ivmeleri neden birlikte çözülür?

T = 1/2 * nu.T @ M @ nu. Caster'ın ötelenmesi ve mutlak swivel ataleti
gövde yaw'ını ve göreli caster dönmesini aynı kütle matrisinde bağlar.
"""
import numpy as np
from ..state import VelocityIndex as V


def mass_matrix_and_bias(state, parameters, geometry):
    g, d = parameters.geometry, parameters.dynamics
    matrix = np.diag([d.body_lumped_mass_kg, d.body_lumped_mass_kg, d.body_yaw_inertia_kgm2,
                      0.0, d.drive_roll_inertia_kgm2, d.drive_roll_inertia_kgm2, d.caster_roll_inertia_kgm2])
    caster_map = geometry.caster_velocity_jacobian
    matrix += d.caster_lumped_mass_kg*(caster_map.T @ caster_map)
    absolute_swivel_map = np.zeros(7)
    absolute_swivel_map[[V.BODY_YAW, V.CASTER_RELATIVE_SWIVEL]] = 1
    matrix += d.caster_swivel_inertia_com_kgm2*np.outer(absolute_swivel_map, absolute_swivel_map)
    centripetal = (-geometry.pivot_from_com_world_m*state.body_yaw_rate_radps**2
                   + g.caster_trail_m*geometry.caster_forward*state.caster_world_rate_radps**2)
    bias = d.caster_lumped_mass_kg*(caster_map.T @ centripetal)
    return matrix, bias
