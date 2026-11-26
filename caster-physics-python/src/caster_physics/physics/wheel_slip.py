"""OKUMA 5 — Merkez ilerleme hızı ile çevresel yuvarlanma hızı uyuşuyor mu?

s_long = e·v_merkez - r*omega; s_lat = n·v_merkez [m/s].
Kayma hızı sıfırsa bu yumuşak temas yasası çekiş kuvvetini sıfır verir.
"""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class WheelSlips:
    left_mps: np.ndarray       # [boyuna, yanal]
    right_mps: np.ndarray
    caster_mps: np.ndarray


def wheel_slips(state, parameters, geometry):
    g = parameters.geometry
    velocity = state.velocities()

    def slip(jacobian, forward, lateral, radius, roll_rate):
        center_velocity = jacobian @ velocity
        return np.array([forward @ center_velocity-radius*roll_rate, lateral @ center_velocity])

    return WheelSlips(
        slip(geometry.left_velocity_jacobian, geometry.drive_forward, geometry.drive_lateral,
             g.drive_radius_m, state.left_wheel_rate_radps),
        slip(geometry.right_velocity_jacobian, geometry.drive_forward, geometry.drive_lateral,
             g.drive_radius_m, state.right_wheel_rate_radps),
        slip(geometry.caster_velocity_jacobian, geometry.caster_forward, geometry.caster_lateral,
             g.caster_radius_m, state.caster_roll_rate_radps),
    )
