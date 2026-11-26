"""OKUMA 1 — Robot hakkında hangi büyüklükleri zamanla izliyoruz?

13 durum = 4 konum/açı + 7 hız + 2 enerji integrali.
Bu dosya sayısal çözücü vektörünün sırasını bilen tek yerdir.
"""
from dataclasses import dataclass, fields
from enum import IntEnum

import numpy as np


class VelocityIndex(IntEnum):
    """Yedi bağlı hareket denklemindeki satır/sütunların fiziksel adları."""
    WORLD_X = 0
    WORLD_Y = 1
    BODY_YAW = 2
    CASTER_RELATIVE_SWIVEL = 3
    LEFT_WHEEL_ROLL = 4
    RIGHT_WHEEL_ROLL = 5
    CASTER_WHEEL_ROLL = 6


@dataclass(frozen=True)
class State:
    body_x_m: float = 0.0                    # B: gövde COM, dünya X
    body_y_m: float = 0.0                    # B: gövde COM, dünya Y
    body_yaw_rad: float = 0.0                # theta: saat yönü tersi pozitif
    caster_angle_relative_rad: float = 0.0   # phi: gövdeye göre caster açısı
    body_velocity_x_mps: float = 0.0         # dünya hızı; gövde boyuna hızı değil
    body_velocity_y_mps: float = 0.0
    body_yaw_rate_radps: float = 0.0
    caster_relative_rate_radps: float = 0.0
    left_wheel_rate_radps: float = 0.0
    right_wheel_rate_radps: float = 0.0
    caster_roll_rate_radps: float = 0.0
    motor_work_J: float = 0.0                # integral(tau_L*w_L + tau_R*w_R)
    dissipated_energy_J: float = 0.0         # sürtünmelerde kaybolan enerji

    @property
    def caster_world_angle_rad(self):
        return self.body_yaw_rad + self.caster_angle_relative_rad

    @property
    def caster_world_rate_radps(self):
        return self.body_yaw_rate_radps + self.caster_relative_rate_radps

    def velocities(self):
        return np.array([
            self.body_velocity_x_mps, self.body_velocity_y_mps,
            self.body_yaw_rate_radps, self.caster_relative_rate_radps,
            self.left_wheel_rate_radps, self.right_wheel_rate_radps,
            self.caster_roll_rate_radps,
        ])

    def as_vector(self):
        return np.array([getattr(self, field.name) for field in fields(self)], dtype=float)

    @classmethod
    def from_vector(cls, vector):
        vector = np.asarray(vector, dtype=float)
        if vector.shape != (13,) or not np.all(np.isfinite(vector)):
            raise ValueError("Durum tam 13 sonlu SI değeri içermeli.")
        return cls(*vector)

    @classmethod
    def at_rest(cls, parameters):
        # A aks merkezi (0,0) iken B COM, gövde COM ofsetindedir.
        return cls(body_x_m=parameters.dynamics.body_com_m[0],
                   body_y_m=parameters.dynamics.body_com_m[1])


@dataclass(frozen=True)
class Torques:
    """Tekerlek aksına uygulanan torklar [N m]; hız veya voltaj değildir."""
    left_Nm: float
    right_Nm: float

    def __post_init__(self):
        if not np.all(np.isfinite([self.left_Nm, self.right_Nm])):
            raise ValueError("Torklar sonlu olmalı.")


def derivative_vector(state, accelerations, motor_power_W, dissipation_power_W):
    """Konum türevi hızdır; hız türevi ivmedir; iş türevi güçtür."""
    return np.concatenate(([
        state.body_velocity_x_mps, state.body_velocity_y_mps,
        state.body_yaw_rate_radps, state.caster_relative_rate_radps,
    ], accelerations, [motor_power_W, dissipation_power_W]))
