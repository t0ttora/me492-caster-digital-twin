"""Analytic, no-slip open-loop calculation driven by contract velocity samples."""
import math

from .commands import VELOCITY


def ideal_velocity_replay(series, grid_s, radius, track, origin=(0.0, 0.0, 0.0)):
    if series.schema != VELOCITY:
        raise ValueError("velocity only: axle torque requires an effort-actuated plant")
    if not all(math.isfinite(x) and x > 0 for x in (grid_s, radius, track)):
        raise ValueError("grid, radius and track must be finite and positive")

    end = series.time[-1]
    count = math.ceil(end / grid_s)
    if count > 1_000_000:
        raise ValueError("grid exceeds one million samples")
    regular = {min(round(index * grid_s, 12), end) for index in range(count + 1)}
    # Include command edges so no integration interval spans a command change.
    times = sorted(regular.union(series.time))
    x, y, yaw = origin
    path_m = 0.0
    previous_time = 0.0
    previous_v, previous_w = series.hold(0.0)
    for time_s in times:
        dt = time_s - previous_time
        next_yaw = yaw + previous_w * dt
        half_turn = 0.5 * previous_w * dt
        arc_scale = math.sin(half_turn) / half_turn if half_turn else 1.0
        x += previous_v * dt * arc_scale * math.cos(yaw + half_turn)
        y += previous_v * dt * arc_scale * math.sin(yaw + half_turn)
        path_m += abs(previous_v) * dt
        yaw = next_yaw
        v, yaw_rate = series.hold(time_s)
        yield {
            "time_s": time_s, "x_m": x, "y_m": y, "yaw_rad": yaw,
            "v_mps": v, "yaw_rate_radps": yaw_rate, "integrated_abs_distance_m": path_m,
            "ideal_left_radps": (v - yaw_rate * track / 2) / radius,
            "ideal_right_radps": (v + yaw_rate * track / 2) / radius,
        }
        previous_time, previous_v, previous_w = time_s, v, yaw_rate
