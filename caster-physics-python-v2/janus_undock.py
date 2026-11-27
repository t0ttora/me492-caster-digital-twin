"""Bounded undock policy for the synthetic station-release fixture."""
from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class UndockBudget:
    release_timeout_s: float = .80
    release_freshness_s: float = .20
    reverse_request_mps: float = -.12
    reverse_distance_m: float = .20
    reverse_timeout_s: float = 3.
    wall_bounds_m: tuple = (-.32, .91, -.34, .34)


class Undock:
    def __init__(self, start_x_m):
        self.budget = UndockBudget()
        self.start_x_m = start_x_m
        self.authorized_s = self.release_s = self.reverse_s = None
        self.charging = self.contact = True
        self.phase = "hold"
        self.done = False
        self.reason = "await_authorization"

    def configuration(self):
        return {**asdict(self.budget), "release_source": "virtual_bms_and_contact_fixture"}

    def authorize(self, t):
        if self.authorized_s is None and not self.done:
            self.authorized_s = t

    def release(self, t, capture_s):
        if (self.authorized_s is not None and not self.done and self.release_s is None
                and self.authorized_s < capture_s <= t and t-capture_s <= self.budget.release_freshness_s
                and t-self.authorized_s < self.budget.release_timeout_s-1e-10):
            self.release_s = t
            self.charging = self.contact = False
            return True
        return False

    def finish(self, phase, reason):
        self.phase, self.reason, self.done = phase, reason, True
        return 0., 0., self.reason

    def update(self, t, x_m, stopped, interlock=None):
        if interlock:
            return self.finish("abort", interlock)
        if self.done:
            return 0., 0., self.reason
        if self.authorized_s is None:
            return 0., 0., "await_authorization"
        if self.release_s is None:
            if t-self.authorized_s >= self.budget.release_timeout_s-1e-10:
                return self.finish("abort", "virtual_release_timeout")
            return 0., 0., "await_virtual_release"
        if t <= self.release_s or (self.reverse_s is None and not stopped):
            if t-self.release_s >= self.budget.release_timeout_s-1e-10:
                return self.finish("abort", "stationary_release_timeout")
            return 0., 0., "await_stationary_release_sample"
        if self.charging or self.contact:
            return self.finish("abort", "release_hold_lost")
        if self.reverse_s is None:
            self.reverse_s = t
        if self.start_x_m-x_m >= self.budget.reverse_distance_m:
            return self.finish("stop", "backout_distance_reached")
        if t-self.reverse_s >= self.budget.reverse_timeout_s-1e-10:
            return self.finish("abort", "backout_timeout")
        self.phase = "backout"
        return self.budget.reverse_request_mps, 0., "bounded_backout"
