"""Read the state fields used by the guard, exporter and offline fixtures."""
import math

JOINTS = {
    "left_wheel_joint": "left_wheel",
    "right_wheel_joint": "right_wheel",
    "caster_swivel_joint": "caster_swivel",
    "caster_wheel_joint": "caster_roll",
}


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec / 1_000_000_000


def twist_fields(twist):
    return {"v_mps": twist.linear.x, "yaw_rate_radps": twist.angular.z}


def odometry_fields(msg, parent):
    if msg.header.frame_id != parent or msg.child_frame_id != "base_link":
        raise ValueError("unexpected odometry frames")
    position = msg.pose.pose.position
    orient = msg.pose.pose.orientation
    norm = math.hypot(orient.x, orient.y, orient.z, orient.w)
    if not math.isclose(norm, 1.0, abs_tol=0.01):
        raise ValueError("invalid odometry quaternion")
    x, y, z, w = (part / norm for part in (orient.x, orient.y, orient.z, orient.w))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    data = {"time_s": stamp_s(msg.header.stamp), "x_m": position.x, "y_m": position.y,
            "z_m": position.z, "yaw_rad": yaw, **twist_fields(msg.twist.twist)}
    if not all(map(math.isfinite, data.values())):
        raise ValueError("nonfinite odometry")
    return data


def joint_fields(msg):
    if (len(msg.name) != len(set(msg.name)) or set(msg.name) != set(JOINTS)
            or len(msg.position) != len(msg.name) or len(msg.velocity) != len(msg.name)):
        raise ValueError("incomplete or duplicate joint state")
    data = {"time_s": stamp_s(msg.header.stamp)}
    for name, angle, velocity in zip(msg.name, msg.position, msg.velocity):
        joint = JOINTS[name]
        data[f"{joint}_angle_rad"] = angle
        data[f"{joint}_radps"] = velocity
    if not all(map(math.isfinite, data.values())):
        raise ValueError("nonfinite joint state")
    return data
