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
        self.high_water_capture_s = -math.inf
        self.current = Observation("no_frame")

    def update(self, time_s, arrivals):
        for frame in arrivals:
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
            elif len(matching) > 1:
                self.current = Observation("ambiguous_id", frame=frame)
            elif not matching:
                reason = ("wrong_id" if any(item.id for item in frame.candidates)
                          else "undecoded" if frame.candidates else "missing")
                self.current = Observation(reason, frame=frame)
            elif any(not item.id for item in frame.candidates):
                self.current = Observation("ambiguous_undecoded", frame=frame)
            else:
                candidate = matching[0]
                corners = np.asarray(candidate.corners_px)
                center = candidate.center_px
                if not candidate.visible or not math.isfinite(candidate.quality) or candidate.quality < .5:
                    self.current = Observation("not_visible", candidate, frame)
                elif (center is None
                      or corners[3, 1]-corners[0, 1] <= 1. or corners[2, 1]-corners[1, 1] <= 1.):
                    self.current = Observation("invalid_corners", candidate, frame)
                else:
                    error = (center[0]-self.camera.target_u_px)/self.camera.width_px
                    self.current = Observation("valid", candidate, frame, error)
        latest = self.current.frame
        if (latest is not None and self.current.reason not in {"invalid_clock", "disordered", "stale"}
                and time_s-latest.capture_time_s > self.camera.freshness_s+1e-10):
            self.current = Observation("stale", self.current.candidate, latest)
        return self.current


class Dock:
    INNER = .02
    OUTER = .04
    STANDOFF_CAMERA_M = .22
    STANDOFF_AXLE_M = .39
    POSITION_LIMIT_M = .02
    YAW_LIMIT_RAD = math.radians(3.)
    MECHANICAL_DWELL_S = .25
    MECHANICAL_SETTLE_TIMEOUT_S = 1.5
    BMS_FRESHNESS_S = .20
    RETRIES = 2
    CENTER_TIMEOUT_S = 16.
    SEARCH_LIMIT_S = 3.2
    SWEEP_SEARCH_LIMIT_S = 5.2

    def __init__(self, camera, side_m, yaw_gain=4., search_sweep=False, *, search_enabled=False):
        self.side_m = side_m
        self.camera = camera
        self.yaw_gain = yaw_gain
        self.search_sweep = search_sweep
        self.search_enabled = search_enabled or search_sweep
        self.search_limit_s = self.SWEEP_SEARCH_LIMIT_S if search_sweep else self.SEARCH_LIMIT_S
        self.center = Dwell(.4)
        self.stop = Dwell(self.MECHANICAL_DWELL_S)
        self.stop_started_s = None
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

    def abort(self, time_s, reason):
        if self.phase == "abort":
            return 0., 0., self.events[-1]["reason"]
        self.aligned = self.contact = self.charge_confirmed = False
        self.done = True
        return self.transition(time_s, "abort", reason)

    def update(self, time_s, observation, stopped, mechanical, interlock=None, bms_time_s=None):
        if interlock:
            return self.abort(time_s, interlock)
        if self.phase == "stop" or self.contact:
            pos_error, yaw_error = mechanical
            hold = (stopped and pos_error is not None and yaw_error is not None
                    and 0. <= pos_error <= self.POSITION_LIMIT_M and 0. <= yaw_error <= self.YAW_LIMIT_RAD)
            if self.contact and not hold:
                return self.abort(time_s, "mechanical_hold_lost")
        if self.done:
            return 0., 0., self.events[-1]["reason"]
        if self.phase == "wait_for_charge":
            if (bms_time_s is not None and self.contact and self.wait_s < bms_time_s <= time_s
                    and time_s-bms_time_s <= self.BMS_FRESHNESS_S):
                self.charge_confirmed = self.done = True
                return self.transition(time_s, "success", "virtual_charge_confirmed")
            if time_s-self.wait_s >= 1.5-1e-10:
                self.done = True
                return self.transition(time_s, "timeout", "charge_timeout")
            return 0., 0., "await_virtual_bms"
        if self.phase == "stop":
            if self.stop.update(time_s, hold):
                self.aligned = True
                self.events.append(dict(time_s=time_s, event="aligned", reason="synthetic_geometry"))
                self.contact = True
                self.events.append(dict(time_s=time_s, event="contact",
                                        reason="virtual_contact_at_standoff"))
                self.wait_s = time_s
                return self.transition(time_s, "wait_for_charge", "stopped_and_virtual_contact")
            if time_s-self.stop_started_s >= self.MECHANICAL_SETTLE_TIMEOUT_S-1e-10:
                return self.abort(time_s, "mechanical_alignment_proxy_failed")
            return 0., 0., "braking"
        if self.acquire_s is None and time_s >= self.search_limit_s:
            return self.abort(time_s, "search_timeout:"+observation.reason)
        if not observation.valid:
            self.center.update(time_s, False)
            if self.acquire_s is not None:
                if self.loss_s is None:
                    self.loss_s = time_s
                    self.retries += 1
                    self.events.append(dict(time_s=time_s, event="lost", reason=observation.reason))
                if self.retries > self.RETRIES:
                    return self.abort(time_s, "retry_budget")
                if time_s-self.loss_s >= 1.6-1e-10:
                    return self.abort(time_s, "observation_timeout:"+observation.reason)
                return self.transition(time_s, "recenter", observation.reason)
            yaw = 0.
            frame = observation.frame
            if (self.search_enabled and observation.reason == "missing" and frame is not None
                    and 0. <= time_s-frame.capture_time_s <= self.camera.freshness_s+1e-10):
                yaw = (-.55 if time_s < 1.35 else .55) if self.search_sweep else .45
            return 0., yaw, observation.reason

        if self.acquire_s is None or self.loss_s is not None:
            self.events.append(dict(time_s=time_s, event="acquired", reason="fresh_matching_id"))
            self.acquire_s, self.loss_s = time_s, None
            self.transition(time_s, "center", "fresh_matching_id")
        if self.phase in {"center", "recenter"} and time_s-self.acquire_s >= self.CENTER_TIMEOUT_S:
            return self.abort(time_s, "center_timeout")
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
            self.stop_started_s = time_s
            return self.transition(time_s, "stop", "projected_range_threshold")
        speed = min(.14, max(.03, .9*(range_m-self.STANDOFF_CAMERA_M)))
        return speed, yaw_cmd, "tracking_qr"
