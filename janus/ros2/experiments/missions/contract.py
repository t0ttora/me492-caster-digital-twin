"""Lab fixture constants and portable geometry for the JANUS mission cases."""
import math

DOCK_CASES = {"Q15", "Q16", "Q18", "O07"}
CASES = (*sorted(DOCK_CASES), "O01", "O06")
QR_ID = "JANUS-DOCK-01"
QR_SIDE_M = .24 * 21 / 29
QR_STANDOFF_M = .265
QR_FRESHNESS_S = .20
DOCK_TARGET = (1.50 - .25 - QR_STANDOFF_M, 0., 0.)
POS_LIMIT_M = .02
YAW_LIMIT_RAD = .05  # Experiment contract, stricter than the plan's 3°.
SPEED_LIMIT_MPS = .02
YAW_RATE_LIMIT_RADPS = .05
STOP_DWELL_S = .25
DRIVE_RADIUS_M = .075
CHARGE_WAIT_S = 1.50
BMS_FRESHNESS_S = .20
CONTACT_RELEASE_M = .030
UNDOCK_TARGET_X = DOCK_TARGET[0] - .58
CASTER_ERROR_LIMIT_RAD = .15
CASTER_RATE_LIMIT_RADPS = .10
CASTER_DWELL_S = .25
SERVICE_TARGET = (.50, 0., 0.)
NATIVE_CONTACT_LINKS = ("left_wheel", "right_wheel", "caster_wheel")
WAYPOINTS = (("A-in", (0., 0., 0.)), ("B", (.42, 0., 0.)), ("A-out", (0., 0., 0.)))
LOW_SOC_LIMIT = .15
TRANSIT_TARGET = (-.30, 0., 0.)
# Rear face and two flanks of the declared indoor dock fixture.
GUIDES = {
    "dock_back": (1.36, 0., .08, .72),
    "dock_left": (1.16, .335, .32, .08),
    "dock_right": (1.16, -.335, .32, .08),
}


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def pose_error(row, goal):
    return math.hypot(row["x_m"] - goal[0], row["y_m"] - goal[1]), abs(wrap(row["yaw_rad"] - goal[2]))


def in_position(row, goal):
    position, yaw = pose_error(row, goal)
    return position <= POS_LIMIT_M and yaw <= YAW_LIMIT_RAD


def stopped(body, odom, joint):
    return all(abs(row["v_mps"]) <= SPEED_LIMIT_MPS
               and abs(row["yaw_rate_radps"]) <= YAW_RATE_LIMIT_RADPS for row in (body, odom)) and all(
        DRIVE_RADIUS_M * abs(joint[side + "_wheel_radps"]) <= SPEED_LIMIT_MPS
        for side in ("left", "right"))


def zero(row):
    return abs(row["v_mps"]) < 1e-12 and abs(row["yaw_rate_radps"]) < 1e-12


def fresh_bms(capture, receipt, wait):
    return wait < capture <= receipt and receipt - capture <= BMS_FRESHNESS_S


def rectangle(x, y, width, height, yaw=0.):
    # Counterclockwise vertices.
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * u - s * v, y + s * u + c * v)
            for u, v in ((-width/2, -height/2), (width/2, -height/2),
                         (width/2, height/2), (-width/2, height/2))]


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def edges(polygon):
    return zip(polygon, polygon[1:] + polygon[:1])


def point_segment_distance(point, start, end):
    delta = (end[0] - start[0], end[1] - start[1])
    rel = (point[0] - start[0], point[1] - start[1])
    u = max(0., min(1., dot(rel, delta) / dot(delta, delta)))
    return math.hypot(rel[0] - u * delta[0], rel[1] - u * delta[1])


def rectangle_clearance(a, b):
    """Euclidean edge separation, negative minimum separating depth on overlap."""
    def separation(axis):
        ap, bp = [dot(p, axis) for p in a], [dot(p, axis) for p in b]
        return max(min(ap)-max(bp), min(bp)-max(ap))

    gaps = []
    for polygon in (a, b):
        for start, end in list(edges(polygon))[:2]:
            dx, dy = end[0]-start[0], end[1]-start[1]
            length = math.hypot(dx, dy)
            gaps.append(separation((-dy/length, dx/length)))
    if max(gaps) <= 0.:
        return max(gaps)
    return min(point_segment_distance(p, start, end)
               for points, segments in ((a, b), (b, a))
               for p in points for start, end in edges(segments))


def dock_clearance(body, joint, guides=GUIDES):
    """Planar model-collision footprint distance using native base and swivel state."""
    yaw = body["yaw_rad"]
    x, y = body["x_m"], body["y_m"]
    c, s = math.cos(yaw), math.sin(yaw)
    swivel = joint["caster_swivel_angle_rad"]
    components = [
        (0., 0., .50, .32, 0.),              # chassis box
        (0., .19, .15, .03, 0.),            # left drive cylinder envelope
        (0., -.19, .15, .03, 0.),           # right drive cylinder envelope
        (-.18 - .05*math.cos(swivel), -.05*math.sin(swivel), .08, .02, swivel),
    ]
    footprints = [rectangle(x+c*u-s*v, y+s*u+c*v, w, h, yaw+relative_yaw)
                  for u, v, w, h, relative_yaw in components]
    return min(rectangle_clearance(footprint, rectangle(*guide))
               for footprint in footprints for guide in guides.values())
