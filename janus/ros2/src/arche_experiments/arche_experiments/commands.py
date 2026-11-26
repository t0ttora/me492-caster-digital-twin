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

    def __init__(self, command_timeout=0.25, odom_timeout=0.25, clock_timeout=0.50):
        if not all(math.isfinite(t) and t > 0 for t in (command_timeout, odom_timeout, clock_timeout)):
            raise ValueError("guard timeouts must be finite and positive")
        self.command_timeout = command_timeout
        self.odom_timeout = odom_timeout
        self.clock_timeout = clock_timeout
        self.now = self.clock_wall = self.odom_stamp = self.request_stamp = None
        self.fault = None
        self.requested = Command()

    def clock(self, now, wall):
        if self.now is not None and now < self.now:
            self.fault = self.fault or "clock_reset"
        if self.now is None or now != self.now:
            self.clock_wall = wall
        self.now = now

    def odometry(self, stamp, valid=True):
        if not valid or not math.isfinite(stamp) or stamp < 0:
            self.fault = self.fault or "invalid_odom"
            return
        if self.odom_stamp is not None and stamp < self.odom_stamp:
            self.fault = self.fault or "odom_regression"
            return
        self.odom_stamp = stamp

    def request(self, v, yaw):
        if not all(map(math.isfinite, (v, yaw))):
            self.fault = self.fault or "invalid_command"
            return
        self.requested = Command(v, yaw)
        self.request_stamp = self.now

    def abort(self):
        self.fault = self.fault or "operator_abort"

    def output(self, wall):
        if self.fault:
            return Command(reason=self.fault)
        if self.now is None:
            return Command(reason="no_clock")
        if wall - self.clock_wall > self.clock_timeout:
            self.request_stamp = None
            return Command(reason="clock_stall")
        # Header stamps may lead the most recent clock sample due to DDS callback ordering.
        if self.odom_stamp is None or self.now - self.odom_stamp > self.odom_timeout:
            return Command(reason="odom_timeout")
        if self.odom_stamp - self.now > 0.05:
            return Command(reason="future_odom")
        if self.request_stamp is None or self.now - self.request_stamp > self.command_timeout:
            return Command(reason="command_timeout")
        return self.requested
