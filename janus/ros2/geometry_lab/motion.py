"""Reference geometry and isolated test controllers. All distances are in SI."""
import math

STILL_SPEED = .02
STILL_YAW = .05
STILL_DWELL = .25


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, limit):
    return max(-limit, min(limit, value))


def advance(pose, distance, turn):
    x, y, yaw = pose
    if abs(turn) < 1e-10:
        return x + distance*math.cos(yaw), y + distance*math.sin(yaw), yaw + turn
    return (x + distance/turn*(math.sin(yaw+turn)-math.sin(yaw)),
            y + distance/turn*(math.cos(yaw)-math.cos(yaw+turn)), yaw+turn)


def rounded_segments(variant):
    radius, t_straight, t_corner = (.10, 2., 1.7) if variant in {"original", "translated-original"} else (.06, 2.8, 2.4)
    return [(.30, 0., t_straight), (radius*math.pi/2, math.pi/2, t_corner)]*4


def error_body(pose, ref):
    dx, dy = ref[0]-pose[0], ref[1]-pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return dx*c + dy*s, -dx*s + dy*c, wrap(ref[2]-pose[2])


def segment_reference(origin, segment, elapsed):
    distance, turn, duration = segment
    u = max(0., min(1., elapsed/duration))
    return advance(origin, distance*u, turn*u), distance/duration, turn/duration


def rounded_reference(start, segments, elapsed):
    for index, segment in enumerate(segments):
        duration = segment[2]
        if elapsed < duration:
            return index, *segment_reference(start, segment, elapsed)
        start = advance(start, *segment[:2])
        elapsed -= duration
    return len(segments), start, 0., 0.


def follow(pose, target, v, w):
    along, lateral, facing = error_body(pose, target)
    return (clamp(v*math.cos(facing)+2.5*along, .25),
            clamp(w + 3.4*facing + 23.*v*lateral, 1.30))


def terminal(pose, target, reverse=False):
    along, lateral, facing = error_body(pose, target)
    if math.hypot(along, lateral) <= .012:
        return 0., clamp(2.2*facing, .35) if abs(facing) > .024 else 0.
    signed = -1. if reverse else 1.
    bearing = math.atan2(signed*lateral, signed*along)
    return (signed*max(0., math.cos(bearing))*min(.075, 1.8*math.hypot(along, lateral)),
            clamp(2.5*bearing, .50))


def pose_gate(pose, target, speed, yaw_rate):
    return (math.hypot(pose[0]-target[0], pose[1]-target[1]) <= .02
            and abs(wrap(pose[2]-target[2])) <= .05
            and speed <= STILL_SPEED and abs(yaw_rate) <= STILL_YAW)
