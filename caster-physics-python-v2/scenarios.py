"""JANUS 2D fixtures and sample-timed route acceptance on the V2.3 plant."""
from dataclasses import dataclass
from math import atan2, cos, hypot, pi, sin

import numpy as np

from twin import MANEUVERS, Segment, advance_pose, controller_gains, wrap


@dataclass(frozen=True)
class Fixture:
    case: str
    variant: str = "nominal"
    duration_s: float = 14.
    segments: tuple = ()
    gate_each: bool = False
    caster_deg: float = 0.
    profile: str = "baseline"
    comparison: str = "equal_peak"
    surface: str = "reference_synthetic"
    payload_kg: float = 0.
    yaw_deg: float = 0.
    y_m: float = 0.
    seed: int = 492
    events: tuple = ()
    hardware_scope: str = ""
    search_sweep: bool = False
    qr_yaw_gain: float | None = 4.
    search_limit_s: float | None = None
    search_reverse_s: float | None = None
    camera_delays_s: tuple = ()
    axle_x_m: float = 0.
    camera_image_fixture: str = ""
    marker_mount_yaw_deg: float = 0.
    marker_mount_calibration_deg: float = 0.


SQUARE_CW = (Segment(.5, 0., 2.5), Segment(0., -pi/2, 2.4))*4
SQUARE_CCW = (Segment(.5, 0., 2.5), Segment(0., pi/2, 2.4))*4
CORNER = Segment(.1*pi/2, pi/2, 1.7)
ROUND_SQUARE = (Segment(.30, 0., 2.), CORNER)*4
ROUND_SQUARE_NARROW = (Segment(.30, 0., 2.8), Segment(.06*pi/2, pi/2, 2.4))*4
EXTENDED_REVERSAL = (MANEUVERS["reversal"][0], Segment(-1.05, 0., 7.))
P0 = ("M01", "M02", "M04", "M05", "M08", "Q01", "Q02", "Q03", "Q06",
      "Q08", "Q13", "Q15", "Q16", "O04")
STOP_INTERLOCKS = {"obstacle", "command_timeout", "operator_cancel",
                   "odometry_timeout", "odometry_clock_regression"}
INJECTED_STOP_SCOPE = ("Scheduled stop reason and sampled planar braking; request, odometry and encoder "
                       "streams and watchdog detection timing are outside this fixture.")

FIXTURES = (
    Fixture("M01", duration_s=2.),
    Fixture("M02", duration_s=10., segments=MANEUVERS["straight"]),
    Fixture("M02", "brake-step", 7., events=((1.8, "brake_step"),)),
    Fixture("M04", duration_s=13., segments=MANEUVERS["s_turn"], caster_deg=45),
    Fixture("M05", "cw", 85., SQUARE_CW, True),
    Fixture("M06", "ccw", 85., SQUARE_CCW, True),
    Fixture("M08", "caster-plus30", 26., MANEUVERS["reversal"], True, 30),
    Fixture("M08", "caster-minus30", 26., MANEUVERS["reversal"], True, -30),
    Fixture("M08", "caster-plus30-extended", 30., EXTENDED_REVERSAL, True, 30),
    Fixture("M08", "caster-minus30-extended", 30., EXTENDED_REVERSAL, True, -30),
    Fixture("Q01"),
    Fixture("Q02", "left", 20., yaw_deg=-17),
    Fixture("Q03", "right", 20., yaw_deg=17),
    Fixture("Q06", events=((2., "loss"),)),
    Fixture("Q08", events=((0., "wrong"),)),
    Fixture("Q09", "image-mixed-ids", events=((2., "image_replay"), (2.05, "visible")),
            camera_image_fixture="mixed_ids", hardware_scope="offline synthetic image plus sampled planar control"),
    Fixture("Q09", "image-duplicate-target", events=((2., "image_replay"),),
            camera_image_fixture="duplicate_target", hardware_scope="offline synthetic image plus sampled planar control"),
    Fixture("Q10", "image-spot-glare", events=((2., "image_replay"),),
            camera_image_fixture="spot_glare", hardware_scope="offline synthetic image plus sampled planar control"),
    Fixture("Q10", "image-whiteout", events=((2., "image_replay"),),
            camera_image_fixture="whiteout", hardware_scope="offline synthetic image plus sampled planar control"),
    Fixture("Q11", "image-low-light", events=((2., "image_replay"), (2.25, "visible")),
            camera_image_fixture="low_light", hardware_scope="offline synthetic image plus sampled planar control"),
    Fixture("Q11", "image-motion-blur", events=((2., "image_replay"),),
            camera_image_fixture="horizontal_motion_blur", hardware_scope="offline synthetic image plus sampled planar control"),
    Fixture("Q12", "image-occlusion", events=((2., "image_replay"),),
            camera_image_fixture="occlusion", hardware_scope="offline synthetic image plus sampled planar control"),
    Fixture("Q12", "image-cropped", events=((2., "image_replay"),),
            camera_image_fixture="cropped", hardware_scope="offline synthetic image plus sampled planar control"),
    Fixture("Q13", "stale-frame", events=((2., "stale"),)),
    Fixture("Q15"),
    Fixture("Q16", events=((0., "virtual_charge"),)),
    Fixture("O04", "watchdog", events=((1.4, "command_timeout"),), duration_s=7.,
            hardware_scope=INJECTED_STOP_SCOPE),
    Fixture("O04", "operator", events=((1.4, "operator_cancel"),), duration_s=7.,
            hardware_scope=INJECTED_STOP_SCOPE),
    Fixture("O04", "stream-watchdog", duration_s=7.,
            events=((0., "control_stream"), (1.20, "publisher_cut")),
            hardware_scope="synthetic stamped inputs to shared CommandGuard plus planar braking, no DDS"),
    Fixture("O04", "stream-operator", duration_s=7.,
            events=((0., "control_stream"), (1.40, "stream_operator_cancel")),
            hardware_scope="synthetic operator event to shared CommandGuard plus planar braking, no DDS"),
    Fixture("M03", segments=MANEUVERS["arc_turn"], caster_deg=30),
    Fixture("M04", "equal-peak-quintic", 17., MANEUVERS["s_turn"], caster_deg=45, profile="quintic"),
    Fixture("M04", "equal-duration-baseline", 17., MANEUVERS["s_turn"], caster_deg=45,
            comparison="equal_duration"),
    Fixture("M04", "equal-duration-quintic", 17., MANEUVERS["s_turn"], caster_deg=45,
            profile="quintic", comparison="equal_duration"),
    Fixture("M07", "rounded", 30., ROUND_SQUARE, hardware_scope="2D footprint/wall geometry proxy"),
    Fixture("M07", "wider-envelope", 30., ROUND_SQUARE, hardware_scope="2D footprint/wall geometry proxy"),
    Fixture("M07", "rounded-smooth-tracking", 30., ROUND_SQUARE, profile="quintic",
            hardware_scope="same narrow walls and rounded geometry proxy, smoother schedule"),
    Fixture("M07", "narrow-replanned", 30., ROUND_SQUARE_NARROW, y_m=-.0080, axle_x_m=.0484,
            hardware_scope="2D conservative footprint in the original narrow wall envelope"),
    Fixture("M09", "caster-90", segments=(Segment(0., pi/2, 2.4),), caster_deg=90),
    Fixture("M09", "caster-180", segments=(Segment(0., pi/2, 2.4),), caster_deg=180),
    Fixture("M10", "unloaded", segments=MANEUVERS["arc_turn"], caster_deg=30),
    Fixture("M10", "payload-4kg", segments=MANEUVERS["arc_turn"], caster_deg=30, payload_kg=4),
    Fixture("M11", "nominal", segments=(Segment(.6, 0., 4.),)),
    Fixture("M11", "high-drag", segments=(Segment(.6, 0., 4.),), surface="high_drag_synthetic"),
    Fixture("M11", "boundary", segments=(Segment(.6, 0., 4.),), events=((0., "surface_boundary_x_0.30"),)),
    Fixture("M11", "brake-reference", 7., events=((3.4, "brake_step"),),
            hardware_scope="planar reference surface with a reported open-loop brake command"),
    Fixture("M11", "brake-high-drag", 7., surface="high_drag_synthetic", events=((3.4, "brake_step"),),
            hardware_scope="planar higher-drag surface with a reported open-loop brake command"),
    Fixture("M11", "brake-boundary", 7., events=((0., "surface_boundary_x_0.30"), (3.4, "brake_step")),
            hardware_scope="planar surface boundary and subsequent sampled brake command"),
    Fixture("Q04", "lateral", yaw_deg=-12, y_m=.13, hardware_scope="fixed marker and mechanical alignment proxy"),
    Fixture("Q04", "staged-lateral", 34., yaw_deg=-12, y_m=.13, qr_yaw_gain=1.5,
            events=((0., "qr_pose_staging"),),
            hardware_scope="planar QR pose staging with synthetic marker and virtual contact"),
    Fixture("Q05", "acquire", 20., yaw_deg=55),
    Fixture("Q05", "sweep-right", 22., yaw_deg=55, search_sweep=True, qr_yaw_gain=1.5),
    Fixture("Q05", "sweep-left", 22., yaw_deg=-55, search_sweep=True, qr_yaw_gain=1.5),
    Fixture("Q07", events=((2., "loss"), (2.65, "visible"), (0., "virtual_charge"))),
    Fixture("Q13", "stream-cut", events=((2., "stream_cut"),)),
    Fixture("Q13", "duplicate", events=((2., "duplicate"),)),
    Fixture("Q13", "regressive-capture", events=((2., "capture_rewind"),)),
    Fixture("Q14", "obstacle", events=((2., "obstacle"),), hardware_scope="injected obstacle state"),
    Fixture("Q17", events=((2., "loss"), (2.45, "visible"), (3.25, "loss"),
                           (3.70, "visible"), (4.50, "loss"))),
    Fixture("Q18", "virtual-station-backout", duration_s=5., axle_x_m=.61, caster_deg=30,
            events=((.15, "virtual_release_frame"), (.30, "undock_authorization"), (.65, "virtual_release_frame")),
            hardware_scope="synthetic authorization, virtual charge/contact release and planar backout geometry proxy"),
    Fixture("Q19", "offset", yaw_deg=-10, events=((0., "camera_offset"),),
            hardware_scope="fixed camera and marker transform proxy"),
    Fixture("Q19", "staged-calibrated-mount", 34., yaw_deg=-10, qr_yaw_gain=1.5,
            events=((0., "camera_offset"), (0., "qr_pose_staging")), marker_mount_yaw_deg=12.,
            marker_mount_calibration_deg=12.,
            hardware_scope="configured camera and marker yaw offsets with independent planar dock truth"),
    Fixture("Q19", "staged-missing-mount-calibration", 34., yaw_deg=-10, qr_yaw_gain=1.5,
            events=((0., "camera_offset"), (0., "qr_pose_staging")), marker_mount_yaw_deg=12.,
            hardware_scope="intentional omitted marker yaw calibration on the same synthetic geometry"),
    Fixture("Q20", "delay-noise", 20., yaw_deg=-17, events=((0., "delay_noise"),)),
    Fixture("Q20", "delay-noise-damped", 20., yaw_deg=-17, events=((0., "delay_noise"),), qr_yaw_gain=1.5),
    Fixture("Q20", "fixed55-reference", 20., yaw_deg=-17, events=((0., "delay_noise"),),
            camera_delays_s=(.055,)),
    Fixture("Q20", "fixed55-damped", 20., yaw_deg=-17, events=((0., "delay_noise"),),
            camera_delays_s=(.055,), qr_yaw_gain=None),
    Fixture("O01", "return", 27., MANEUVERS["reversal"], True),
    Fixture("O02", "corridor", 42., (Segment(.35, 0., 2.4), Segment(0., pi, 4.),
                                    Segment(.35, 0., 2.4)), True,
            hardware_scope="2D footprint/wall geometry proxy"),
    Fixture("O03", "resume", events=((1.4, "pedestrian"), (2.5, "clear"), (3., "authorized_resume")),
            hardware_scope="injected pedestrian state with geometry proxy"),
    Fixture("O05", "odometry-cut", events=((1.4, "odometry_timeout"),), duration_s=7.,
            hardware_scope=INJECTED_STOP_SCOPE),
    Fixture("O05", "clock-reset", events=((1.4, "odometry_clock_regression"),), duration_s=7.,
            hardware_scope=INJECTED_STOP_SCOPE),
    Fixture("O05", "guard-odometry-cut", duration_s=7.,
            events=((0., "control_stream"), (1.4, "odometry_stream_cut")),
            hardware_scope="synthetic odometry publisher outage through the shared CommandGuard, no DDS"),
    Fixture("O05", "guard-odometry-rewind", duration_s=7.,
            events=((0., "control_stream"), (1.4, "odometry_stamp_rewind")),
            hardware_scope="synthetic regressive odometry stamp through the shared CommandGuard, no DDS"),
    Fixture("O06", "payload-4kg", segments=MANEUVERS["straight"], payload_kg=4),
    Fixture("O07", "low-soc", events=((0., "transit"), (1.4, "low_soc"), (0., "virtual_charge")),
            hardware_scope="injected SoC and virtual charge"),
    Fixture("O08", "synthetic-recovery", duration_s=4.,
            events=((0., "control_stream"), (1.4, "link_down"), (2., "link_restored"),
                    (2.05, "guard_restart"), (2.55, "fresh_authorization"), (2.56, "stale_epoch_request")),
            hardware_scope="synthetic link, guard reset and authorization input, no power cycle or DDS"),
)

# Historical run names keep their pinned configuration for replay.
FIXTURES += tuple(Fixture(case, "damped-regression", duration_s=duration, yaw_deg=yaw, qr_yaw_gain=None)
                  for case, duration, yaw in (("Q01", 14., 0), ("Q02", 20., -17), ("Q03", 20., 17)))
FIXTURES += tuple(Fixture("Q05", variant, 20., yaw_deg=yaw, search_sweep=True,
                          qr_yaw_gain=None, search_limit_s=3.2, search_reverse_s=1.)
                  for variant, yaw in (("bounded-right", 55), ("bounded-left", -55)))

BLOCKED = {
    "M12": "3D ground/obstacle contact, load transfer and vertical clearance require a 3D model.",
}


def axle_state(state, parameters):
    yaw = state.body_yaw_rad
    cx, cy = parameters.dynamics.body_com_m[:2]
    x = state.body_x_m - cos(yaw)*cx + sin(yaw)*cy
    y = state.body_y_m - sin(yaw)*cx - cos(yaw)*cy
    v = cos(yaw)*state.body_velocity_x_mps + sin(yaw)*state.body_velocity_y_mps
    lateral = -sin(yaw)*state.body_velocity_x_mps + cos(yaw)*state.body_velocity_y_mps
    return np.array([x, y, yaw]), v + state.body_yaw_rate_radps*cy, lateral - state.body_yaw_rate_radps*cx


class Dwell:
    def __init__(self, seconds):
        self.seconds = seconds
        self.start_s = None

    def update(self, time_s, valid):
        if not valid:
            self.start_s = None
            return False
        if self.start_s is None:
            self.start_s = time_s
        return time_s - self.start_s >= self.seconds - 1e-10


def terminal_values(pose, target, v, lateral, yaw_rate):
    return (float(np.linalg.norm(target[:2]-pose[:2])), float(abs(wrap(target[2]-pose[2]))),
            hypot(v, lateral), abs(yaw_rate))


def terminal_ok(values, position_m=.02, yaw_rad=.05):
    return all(value <= limit for value, limit in zip(values, (position_m, yaw_rad, .02, .05)))


def segment_duration(segment, fixture):
    factor = 1.875 if fixture.profile == "quintic" or fixture.comparison == "equal_duration" else 1.
    return segment.duration_s*factor


def progress(fraction, duration, profile):
    u = float(np.clip(fraction, 0., 1.))
    if u == 1.:
        return 1., 0.
    if profile == "quintic":
        return u**3*(10.-15.*u+6.*u*u), 30*u*u*(1.-u)**2/duration
    return u, 1./duration


class Route:
    def __init__(self, fixture, parameters, initial_pose):
        self.fixture = fixture
        self.gains = controller_gains(parameters)
        self.origins = [initial_pose]
        for segment in fixture.segments:
            self.origins.append(advance_pose(self.origins[-1], segment.distance_m, segment.yaw_change_rad))
        self.durations = [segment_duration(segment, fixture) for segment in fixture.segments]
        self.segment = 0
        self.segment_start_s = 0.
        self.dwell = Dwell(.25)
        self.terminal_direction = None
        self.align_yaw = False
        self.gates = []
        self.done = False

    def update(self, time_s, pose, v, lateral, yaw_rate):
        if self.done:
            return 0., 0., self.origins[-1], "settled"
        i = self.segment
        elapsed = time_s-self.segment_start_s
        while not self.fixture.gate_each and i < len(self.durations)-1 and elapsed >= self.durations[i]-1e-10:
            self.segment_start_s += self.durations[i]
            self.segment = i = i+1
            elapsed = time_s-self.segment_start_s
        segment, origin, duration = self.fixture.segments[i], self.origins[i], self.durations[i]
        target = self.origins[i+1]
        if elapsed < duration-1e-10:
            p, rate = progress(elapsed/duration, duration, self.fixture.profile)
            reference = advance_pose(origin, p*segment.distance_m, p*segment.yaw_change_rad)
            delta = reference[:2]-pose[:2]
            ex = cos(pose[2])*delta[0]+sin(pose[2])*delta[1]
            ey = -sin(pose[2])*delta[0]+cos(pose[2])*delta[1]
            error = wrap(reference[2]-pose[2])
            speed = rate*segment.distance_m
            return (speed*cos(error)+self.gains["tracking_x"]*ex,
                    rate*segment.yaw_change_rad+self.gains["tracking_y"]*speed*ey
                    +self.gains["tracking_heading"]*sin(error), reference, "tracking")
        values = terminal_values(pose, target, v, lateral, yaw_rate)
        square_corner = self.fixture.case in ("M05", "M06") and i != len(self.durations)-1
        limits = (.05, np.deg2rad(5)) if square_corner else (.02, .05)
        if self.dwell.update(time_s, terminal_ok(values, *limits)):
            kind = "turn" if segment.distance_m == 0 else "edge"
            self.gates.append(dict(time_s=time_s, gate=i+1, kind=kind,
                                   dwell_start_s=self.dwell.start_s, position_error_m=values[0],
                                   yaw_error_rad=values[1], planar_speed_mps=values[2],
                                   yaw_rate_radps=values[3], position_limit_m=limits[0],
                                   yaw_limit_rad=limits[1], target_pose=target.tolist()))
            if i == len(self.durations)-1:
                self.done = True
            else:
                self.segment += 1
                self.segment_start_s = time_s
                self.dwell = Dwell(.25)
                self.terminal_direction = None
                self.align_yaw = False
            return 0., 0., target, "settled" if self.done else "gate_accepted"
        distance = values[0]
        if self.align_yaw and distance > .012:
            self.align_yaw = False
            self.terminal_direction = None
        if distance <= .006 or (segment.distance_m == 0 and distance <= .012):
            self.align_yaw = True
        if self.align_yaw:
            return 0., 3.*sin(wrap(target[2]-pose[2])), target, "align"
        delta = target[:2]-pose[:2]
        bearing = atan2(delta[1], delta[0])
        if self.terminal_direction is None:
            self.terminal_direction = 1. if cos(bearing-pose[2]) >= 0 else -1.
        alpha = wrap(bearing-pose[2]-(pi if self.terminal_direction < 0 else 0.))
        return (2.*distance*cos(alpha)*self.terminal_direction, 3.*alpha+2.*sin(alpha)*cos(alpha),
                target, "position")
