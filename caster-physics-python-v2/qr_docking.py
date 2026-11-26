"""Signed image feedback and interlocks for the JANUS station fixture."""
from dataclasses import dataclass
import math

import numpy as np

from qr_camera import DOCK_ID, estimate_marker
from scenarios import Dwell


@dataclass(frozen=True)
class Observation:
    reason: str
    candidate: object = None
    frame: object = None
    error: float | None = None

    @property
    def valid(self):
        return self.reason == "valid"


class FrameGate:
    def __init__(self, camera):
        self.camera = camera
        self.frame = None
        self.high_water_capture_s = -math.inf
        self.current = Observation("no_frame")

    def update(self, time_s, arrivals):
        for frame in arrivals:
            self.frame = frame
            ordered = frame.capture_time_s > self.high_water_capture_s
            if not (0 <= frame.capture_time_s <= frame.arrival_time_s <= time_s+1e-10):
                self.current = Observation("invalid_clock", frame=frame)
                continue
            self.high_water_capture_s = max(frame.capture_time_s, self.high_water_capture_s)
            matching = [item for item in frame.candidates if item.id == DOCK_ID]
            if time_s-frame.capture_time_s > self.camera.freshness_s+1e-10:
                self.current = Observation("stale", frame=frame)
            elif not ordered:
                self.current = Observation("disordered", frame=frame)
            elif len(matching) != 1:
                reason = "ambiguous_id" if matching else ("wrong_id" if frame.candidates else "missing")
                self.current = Observation(reason, frame=frame)
            else:
                candidate = matching[0]
                corners = np.asarray(candidate.corners_px)
                if not candidate.visible or not math.isfinite(candidate.quality) or candidate.quality < .5:
                    self.current = Observation("not_visible", candidate, frame)
                elif (corners.shape != (4, 2) or not np.isfinite(corners).all()
                      or corners[3, 1]-corners[0, 1] <= 1. or corners[2, 1]-corners[1, 1] <= 1.):
                    self.current = Observation("invalid_corners", candidate, frame)
                else:
                    error = (candidate.center_px[0]-self.camera.target_u_px)/self.camera.width_px
                    self.current = Observation("valid", candidate, frame, error)
        if self.current.valid and time_s-self.frame.capture_time_s > self.camera.freshness_s+1e-10:
            self.current = Observation("stale", self.current.candidate, self.frame)
        return self.current


class Dock:
    INNER = .02
    OUTER = .04
    STANDOFF_CAMERA_M = .22
    STANDOFF_AXLE_M = .39
    BMS_FRESHNESS_S = .20
    RETRIES = 2
    CENTER_TIMEOUT_S = 16.

    def __init__(self, camera, side_m, yaw_gain=4., search_sweep=False):
        self.side_m = side_m
        self.camera = camera
        self.yaw_gain = yaw_gain
        self.search_sweep = search_sweep
        self.center = Dwell(.4)
        self.stop = Dwell(.25)
        self.phase = "search"
        self.events = []
        self.loss_s = None
        self.acquire_s = None
        self.wait_s = None
        self.retries = 0
        self.aligned = False
        self.contact = False
        self.charge_confirmed = False
        self.done = False

    def transition(self, time_s, phase, reason):
        if phase != self.phase:
            self.events.append(dict(time_s=time_s, event=phase, reason=reason))
            self.phase = phase
        return 0., 0., reason

    def update(self, time_s, observation, stopped, mechanical, interlock=None, bms_time_s=None):
        if self.done:
            return 0., 0., self.events[-1]["reason"]
        if interlock:
            self.done = True
            return self.transition(time_s, "abort", interlock)
        if self.phase == "wait_for_charge":
            if (bms_time_s is not None and self.contact and stopped
                    and self.wait_s <= bms_time_s <= time_s
                    and time_s-bms_time_s <= self.BMS_FRESHNESS_S):
                self.charge_confirmed = self.done = True
                return self.transition(time_s, "success", "virtual_charge_confirmed")
            if time_s-self.wait_s >= 1.5-1e-10:
                self.done = True
                return self.transition(time_s, "timeout", "charge_timeout")
            return 0., 0., "await_virtual_bms"
        if self.phase == "stop":
            if self.stop.update(time_s, stopped):
                pos_error, yaw_error = mechanical
                self.aligned = pos_error <= .02 and yaw_error <= math.radians(3)
                if not self.aligned:
                    self.done = True
                    return self.transition(time_s, "abort", "mechanical_alignment_proxy_failed")
                self.events.append(dict(time_s=time_s, event="aligned", reason="synthetic_geometry"))
                self.contact = True
                self.events.append(dict(time_s=time_s, event="contact",
                                        reason="virtual_contact_at_standoff"))
                self.wait_s = time_s
                return self.transition(time_s, "wait_for_charge", "stopped_and_virtual_contact")
            return 0., 0., "braking"
        if not observation.valid:
            self.center.update(time_s, False)
            if self.acquire_s is not None:
                if self.loss_s is None:
                    self.loss_s = time_s
                    self.retries += 1
                    self.events.append(dict(time_s=time_s, event="lost", reason=observation.reason))
                if self.retries > self.RETRIES:
                    self.done = True
                    return self.transition(time_s, "abort", "retry_budget")
                if time_s-self.loss_s >= 1.6-1e-10:
                    self.done = True
                    return self.transition(time_s, "abort", "observation_timeout:"+observation.reason)
                return self.transition(time_s, "recenter", observation.reason)
            search_limit = 5.2 if self.search_sweep else 3.2
            if time_s >= search_limit:
                self.done = True
                return self.transition(time_s, "abort", "search_timeout:"+observation.reason)
            yaw = 0.
            if observation.reason == "not_visible":
                yaw = (-.55 if time_s < 1.35 else .55) if self.search_sweep else .45
            return 0., yaw, observation.reason

        if self.acquire_s is None or self.loss_s is not None:
            self.events.append(dict(time_s=time_s, event="acquired", reason="fresh_matching_id"))
            self.acquire_s, self.loss_s = time_s, None
            self.transition(time_s, "center", "fresh_matching_id")
        if self.phase in {"center", "recenter"} and time_s-self.acquire_s >= self.CENTER_TIMEOUT_S:
            self.done = True
            return self.transition(time_s, "abort", "center_timeout")
        error = observation.error
        yaw_cmd = max(-.75, min(.75, -self.yaw_gain*error))
        if self.phase == "advance" and abs(error) > self.OUTER:
            self.center.update(time_s, False)
            return self.transition(time_s, "recenter", "outside_outer_band")
        if self.phase != "advance":
            if not self.center.update(time_s, abs(error) <= self.INNER):
                return 0., yaw_cmd, "centering"
            self.transition(time_s, "advance", "center_dwell_met")
        range_m, _ = estimate_marker(observation.candidate, self.camera, self.side_m)
        if range_m <= self.STANDOFF_CAMERA_M+.006:
            return self.transition(time_s, "stop", "projected_range_threshold")
        speed = min(.14, max(.03, .9*(range_m-self.STANDOFF_CAMERA_M)))
        return speed, yaw_cmd, "tracking_qr"
