"""Level pinhole-camera fixture. Project ordered world corners on every capture."""
from dataclasses import dataclass
from math import atan2, cos, hypot, sin

import numpy as np


DOCK_ID = "JANUS-DOCK-01"


@dataclass(frozen=True)
class Camera:
    width_px: int = 640
    height_px: int = 480
    fx_px: float = 510.
    fy_px: float = 510.
    cx_px: float = 320.
    cy_px: float = 240.
    forward_m: float = .17
    left_m: float = 0.
    height_m: float = .22
    yaw_rad: float = 0.
    target_u_px: float = 320.
    frame_period_s: float = .05
    freshness_s: float = .20


@dataclass(frozen=True)
class Marker:
    x_m: float = 1.
    y_m: float = 0.
    height_m: float = .22
    yaw_rad: float = 0.
    side_m: float = .10


@dataclass(frozen=True)
class Candidate:
    id: str
    corners_px: tuple
    visible: bool
    quality: float

    @property
    def center_px(self):
        if not self.corners_px:
            return None
        return tuple(float(x) for x in np.mean(self.corners_px, axis=0))


@dataclass(frozen=True)
class Frame:
    capture_time_s: float
    arrival_time_s: float
    candidates: tuple
    source: str = "projection"


def project(pose, camera, marker, identity=DOCK_ID, rng=None):
    x, y, yaw = pose
    cx = x+cos(yaw)*camera.forward_m-sin(yaw)*camera.left_m
    cy = y+sin(yaw)*camera.forward_m+cos(yaw)*camera.left_m
    optical_yaw = yaw+camera.yaw_rad
    corners = []
    for across, height in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
        ym = across*marker.side_m/2
        dx, dy = marker.x_m-sin(marker.yaw_rad)*ym-cx, marker.y_m+cos(marker.yaw_rad)*ym-cy
        forward = cos(optical_yaw)*dx+sin(optical_yaw)*dy
        right = sin(optical_yaw)*dx-cos(optical_yaw)*dy
        if forward <= .02:
            return Candidate(identity, (), False, 0.)
        down = camera.height_m-marker.height_m-height*marker.side_m/2
        corners.append((camera.cx_px+camera.fx_px*right/forward,
                        camera.cy_px+camera.fy_px*down/forward))
    if rng is not None:
        corners = [tuple(p) for p in np.array(corners)+rng.normal(0., .9, (4, 2))]
    visible = all(0 <= u < camera.width_px and 0 <= v < camera.height_px for u, v in corners)
    front = cos(marker.yaw_rad)*(marker.x_m-cx)+sin(marker.yaw_rad)*(marker.y_m-cy) > 0
    return Candidate(identity, tuple(corners), visible and front, 1. if visible and front else 0.)


def estimate_marker(candidate, camera, side_m):
    """Level rectangular marker: reconstruct each vertical edge from its image height."""
    tl, tr, br, bl = candidate.corners_px

    def horizontal(top, bottom):
        depth = camera.fy_px*side_m/(bottom[1]-top[1])
        right = ((top[0]+bottom[0])/2-camera.cx_px)*depth/camera.fx_px
        return np.array([depth, -right])

    left, right = horizontal(tl, bl), horizontal(tr, br)
    forward, lateral = (left+right)/2
    delta = left-right
    orientation = atan2(-delta[0], delta[1])
    return float(hypot(forward, lateral)), float(orientation)


def mechanical_errors(pose, marker, axle_standoff_m):
    """Separate synthetic truth for acceptance, expressed in the station frame."""
    dx = pose[0]-(marker.x_m-axle_standoff_m*cos(marker.yaw_rad))
    dy = pose[1]-(marker.y_m-axle_standoff_m*sin(marker.yaw_rad))
    yaw_error = atan2(sin(pose[2]-marker.yaw_rad), cos(pose[2]-marker.yaw_rad))
    return hypot(dx, dy), abs(yaw_error)
