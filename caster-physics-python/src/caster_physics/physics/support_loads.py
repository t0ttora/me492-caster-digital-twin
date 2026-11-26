"""OKUMA 4 — Ağırlık üç tekerleğe nasıl dağılır?

ΣN=mg ve zemindeki x/y moment dengeleri. Düşey ivme ve ivmeden yük transferi yok.
"""
import numpy as np


def support_loads(state, parameters):
    g, d = parameters.geometry, parameters.dynamics
    phi = state.caster_angle_relative_rad
    caster_x = g.caster_pivot_m[0]-g.caster_trail_m*np.cos(phi)
    caster_y = g.caster_pivot_m[1]-g.caster_trail_m*np.sin(phi)
    body_mass, caster_mass = d.body_lumped_mass_kg, d.caster_lumped_mass_kg
    caster_normal = d.gravity_mps2*(body_mass*d.body_com_m[0]+caster_mass*caster_x)/caster_x
    left_minus_right = (
        2*d.gravity_mps2*(body_mass*d.body_com_m[1]+caster_mass*caster_y)/g.drive_track_m
        - 2*caster_normal*caster_y/g.drive_track_m
    )
    drive_normal_sum = (body_mass+caster_mass)*d.gravity_mps2-caster_normal
    normal = np.array([.5*(drive_normal_sum+left_minus_right),
                       .5*(drive_normal_sum-left_minus_right), caster_normal])
    if not np.all(np.isfinite(normal)) or np.min(normal) <= 0:
        raise ValueError("COM destek üçgeninin dışında; tekerlek yükleri pozitif değil.")
    return normal
