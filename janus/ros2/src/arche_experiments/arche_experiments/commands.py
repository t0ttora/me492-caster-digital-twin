"""Command CSV contract, zero-order hold, and reusable simulation command guard."""

from bisect import bisect_right
import csv
from dataclasses import dataclass
import math

VELOCITY = ("time_s", "v_mps", "yaw_rate_radps")
TORQUE = ("time_s", "tau_left_Nm", "tau_right_Nm")


@dataclass(frozen=True)
class CommandSeries:
    schema: tuple
    time: tuple
    values: tuple

    def hold(self, time_s):
        if not self.time[0] <= time_s <= self.time[-1]:
            raise ValueError("sample lies outside the command recording")
        return self.values[bisect_right(self.time, time_s) - 1]


def load_commands(path):
    sample_times, values = [], []
    with open(path, newline="") as stream:
        reader = csv.reader(stream)
        schema = tuple(next(reader, ()))
        if schema not in (VELOCITY, TORQUE):
            raise ValueError(f"{path}: expected one of {VELOCITY}, {TORQUE}, got {schema}")
        for line, raw in enumerate(reader, 2):
            if len(raw) != 3:
                raise ValueError(f"{path}:{line}: expected three values")
            time_s, first, second = map(float, raw)
            if not all(map(math.isfinite, (time_s, first, second))):
                raise ValueError(f"{path}:{line}: non-finite value")
            if (sample_times and time_s <= sample_times[-1]) or (not sample_times and time_s != 0):
                raise ValueError(f"{path}:{line}: time must start at 0 and increase strictly")
            sample_times.append(time_s)
            values.append((first, second))
    if len(sample_times) < 2:
        raise ValueError(f"{path}: need at least two timestamps")
    return CommandSeries(schema, tuple(sample_times), tuple(values))


@dataclass(frozen=True)
class Command:
    v_mps: float = 0.0
    yaw_rate_radps: float = 0.0
    reason: str = "ready"


class CommandGuard:
    """Pure guard shared by the ROS adapter and synthetic replay."""

    def __init__(self, command_timeout=0.25, odom_timeout=0.25, clock_timeout=0.50,
                 require_recovery_permission=False):
        if not all(math.isfinite(t) and t > 0 for t in (command_timeout, odom_timeout, clock_timeout)):
            raise ValueError("guard timeouts must be finite and positive")
        self.command_timeout = command_timeout
        self.odom_timeout = odom_timeout
        self.clock_timeout = clock_timeout
        self.now = self.clock_wall = self.odom_stamp = self.request_stamp = None
        self.fault = None
        self.requested = Command()
        self.require_recovery_permission = require_recovery_permission
        self.recovery_pending = False
        self.had_motion_request = False

    def _invalidate_for_watchdog(self):
        if self.require_recovery_permission and self.had_motion_request:
            self.recovery_pending = True
        self.request_stamp = None

    def clock(self, now, wall):
        # Detect a wall-time stall even if the timer did not run during the gap.
        if self._watchdog_reason(wall):
            self._invalidate_for_watchdog()
        if self.now is not None and now < self.now:
            self.fault = self.fault or "clock_reset"
        if self.now is None or now != self.now:
            self.clock_wall = wall
        self.now = now
        if self._odom_watchdog_reason():
            self._invalidate_for_watchdog()

    def odometry(self, stamp, valid=True):
        if not valid or not math.isfinite(stamp) or stamp < 0:
            self.fault = self.fault or "invalid_odom"
            return
        if self.odom_stamp is not None and stamp < self.odom_stamp:
            self.fault = self.fault or "odom_regression"
            return
        self.odom_stamp = stamp
        if self.now is not None and self._odom_watchdog_reason():
            self._invalidate_for_watchdog()

    def request(self, v, yaw, wall):
        if not all(map(math.isfinite, (v, yaw))):
            self.fault = self.fault or "invalid_command"
            return
        if self.fault or self.recovery_pending:
            self.request_stamp = None
            return
        if self._watchdog_reason(wall):
            self._invalidate_for_watchdog()
            return
        self.requested = Command(v, yaw)
        self.request_stamp = self.now
        if v != 0 or yaw != 0:
            self.had_motion_request = True

    def authorize_recovery(self, wall):
        if not self.recovery_pending or self.fault or self._watchdog_reason(wall):
            return False
        self.recovery_pending = False
        self.request_stamp = None
        return True

    def abort(self):
        self.fault = self.fault or "operator_abort"

    def _odom_watchdog_reason(self):
        if self.odom_stamp is None or self.now - self.odom_stamp > self.odom_timeout:
            return "odom_timeout"
        # Header stamps may lead the most recent clock sample due to DDS callback ordering.
        if self.odom_stamp - self.now > 0.05:
            return "future_odom"

    def _watchdog_reason(self, wall):
        if self.now is None:
            return "no_clock"
        if wall - self.clock_wall > self.clock_timeout:
            return "clock_stall"
        return self._odom_watchdog_reason()

    def output(self, wall):
        if self.fault:
            return Command(reason=self.fault)
        if reason := self._watchdog_reason(wall):
            self._invalidate_for_watchdog()
            return Command(reason=reason)
        if self.recovery_pending:
            return Command(reason="recovery_permission_required")
        if self.request_stamp is None or self.now - self.request_stamp > self.command_timeout:
            return Command(reason="command_timeout")
        return self.requested
