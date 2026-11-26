"""OKUMA 3 — Tekerlek merkezleri nerede; gövde hareketi onları nasıl taşır?

J @ hızlar = tekerlek merkezinin dünya düzlemindeki hızı.
J bir hız haritasıdır (Jacobian); kendi başına kuvvet veya ivme üretmez.
"""
from dataclasses import dataclass
import numpy as np

from ..state import VelocityIndex as V


def rotate_90(vector):
    return np.array([-vector[1], vector[0]])


@dataclass(frozen=True)
class RobotGeometry:
    body_rotation: np.ndarray
    drive_forward: np.ndarray
    drive_lateral: np.ndarray
    caster_forward: np.ndarray
    caster_lateral: np.ndarray
    pivot_from_com_world_m: np.ndarray
    caster_from_com_world_m: np.ndarray
    left_velocity_jacobian: np.ndarray
    right_velocity_jacobian: np.ndarray
    caster_velocity_jacobian: np.ndarray


def geometry_at(state, parameters):
    g, d = parameters.geometry, parameters.dynamics
    theta, psi = state.body_yaw_rad, state.caster_world_angle_rad
    rotation = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    drive_forward, drive_lateral = rotation[:, 0], rotation[:, 1]
    caster_forward = np.array([np.cos(psi), np.sin(psi)])
    caster_lateral = rotate_90(caster_forward)
    com_x, com_y, _ = d.body_com_m
    left_offset = rotation @ np.array([-com_x, g.drive_track_m/2-com_y])
    right_offset = rotation @ np.array([-com_x, -g.drive_track_m/2-com_y])
    pivot_offset = rotation @ (np.array(g.caster_pivot_m[:2])-np.array(d.body_com_m[:2]))
    caster_offset = pivot_offset-g.caster_trail_m*caster_forward

    def velocity_map(offset):
        jacobian = np.zeros((2, 7))
        jacobian[:, :2] = np.eye(2)  # COM'un dünya X/Y hızları
        jacobian[:, V.BODY_YAW] = rotate_90(offset)
        return jacobian

    left_map, right_map, caster_map = [velocity_map(offset) for offset in (left_offset, right_offset, caster_offset)]
    caster_map[:, V.CASTER_RELATIVE_SWIVEL] = -g.caster_trail_m*caster_lateral
    return RobotGeometry(rotation, drive_forward, drive_lateral, caster_forward, caster_lateral,
                         pivot_offset, caster_offset, left_map, right_map, caster_map)
