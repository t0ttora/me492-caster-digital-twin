"""Sample-boundary and interlock regressions for the requalification fixtures."""
from math import radians
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from janus_command_input import CommandInput
from janus_metrics import wall_clearances
from janus_sim import CameraStream, parameters_for, qr_staging_route
from janus_undock import Undock
from qr_camera import Camera, Frame, Marker, project
from qr_docking import Dock, FrameGate
from scenarios import FIXTURES


def test_invalid_and_valid_arrivals_in_one_tick_cut_advance_until_a_new_capture():
    camera, marker = Camera(), Marker()
    good = project((0., 0., 0.), camera, marker)
    gate, dock = FrameGate(camera), Dock(camera, marker.side_m)

    def update(t, arrivals):
        return dock.update(t, gate.update(t, arrivals), True, (1., 0.))

    for tick in range(9):
        t = tick*.05
        motion = update(t, [Frame(t, t, (good,))])
    assert motion[0] > 0.
    assert update(.45, [Frame(.44, .45, ()), Frame(.45, .45, (good,))])[:2] == (0., 0.)
    assert dock.retries == 1
    assert update(.46, [])[:2] == (0., 0.)
    for tick in range(10, 18):
        t = tick*.05
        assert update(t, [Frame(t, t, (good,))])[0] == 0.
    assert update(.90, [Frame(.90, .90, (good,))])[0] > 0.


def test_transport_delivers_in_arrival_order_even_when_captures_are_reordered():
    camera, marker = Camera(), Marker()
    stream = CameraStream(camera, marker, 492, True, (.125, 0.))
    gate = FrameGate(camera)
    pose = (0., 0., 0.)
    for tick in range(14):
        t = tick/100
        result = gate.update(t, stream.sample(tick, t, .01, pose, "visible"))
        if tick == 5:
            assert result.valid and result.frame.capture_time_s == .05
    assert [frame.capture_time_s for frame in stream.frames] == [.05, 0.]
    assert result.reason == "disordered"


def test_camera_capture_rewind_stays_fresh_but_fails_strict_ordering():
    camera, marker = Camera(), Marker()
    stream = CameraStream(camera, marker, 492, False)
    gate = FrameGate(camera)
    pose = (0., 0., 0.)
    for tick in (0, 5):
        time = tick/100
        assert gate.update(time, stream.sample(tick, time, .01, pose, "visible")).valid
    bad = stream.sample(10, .10, .01, pose, "capture_rewind")
    assert bad[0].capture_time_s == .04 and bad[0].arrival_time_s == .10
    assert gate.update(.10, bad).reason == "disordered"


def test_staging_pose_accounts_for_calibrated_camera_and_marker_mounts():
    fixture = next(f for f in FIXTURES if (f.case, f.variant) == ("Q19", "staged-calibrated-mount"))
    camera = Camera(left_m=.035, yaw_rad=radians(4))
    marker = Marker(yaw_rad=radians(12))
    pose = np.array([0., 0., radians(-10)])
    candidate = project(pose, camera, marker)
    frame = Frame(0., 0., (candidate,))
    observed = FrameGate(camera).update(0., [frame])
    route = qr_staging_route(fixture, parameters_for(fixture), pose, camera, observed, marker.side_m)
    assert observed.valid
    assert np.allclose(route.origins[-1], [.45, 0., 0.], atol=1e-12)


def test_mounted_yaw_trim_has_zero_linear_command_and_stops_on_stale_frame():
    camera, marker = Camera(), Marker()
    dock, gate = Dock(camera, marker.side_m), FrameGate(camera)
    dock.phase, dock.stop_started_s = "stop", 0.
    candidate = project((0., 0., 0.), camera, marker)
    fresh = gate.update(0., [Frame(0., 0., (candidate,))])
    assert dock.update(0., fresh, False, (.01, .12), final_yaw_error_rad=.12) == (0., .08, "mounted_yaw_trim")
    stale = gate.update(.25, [])
    assert dock.update(.25, stale, False, (.01, .12), final_yaw_error_rad=.12) == (0., 0., "braking")


def test_open_corridor_rear_leaves_front_and_both_side_walls_active():
    positions = np.asarray([[-1., 0.], [-1., .20], [.51, 0.]])
    clearances = wall_clearances(positions, np.zeros(3), (None, .70, -.34, .34))
    assert np.all(np.isposinf(clearances[:, 0]))
    assert np.allclose(clearances.min(axis=1), [.135, -.065, -.06])


def test_bounded_search_reversal_and_interlock_at_sample_boundary():
    camera, marker = Camera(), Marker()
    gate = FrameGate(camera)
    dock = Dock(camera, marker.side_m, search_sweep=True, search_limit_s=3.2, search_reverse_s=1.)
    before = gate.update(.99, [Frame(.99, .99, ())])
    after = gate.update(1., [Frame(1., 1., ())])
    assert dock.update(.99, before, False, (1., 0.))[:2] == (0., -.55)
    assert dock.update(1., after, False, (1., 0.))[:2] == (0., .55)
    assert dock.update(1.01, after, False, (1., 0.), interlock="operator_cancel") == (0., 0., "operator_cancel")
    assert dock.update(1.02, after, False, (1., 0.)) == (0., 0., "operator_cancel")


def test_center_watchdog_cuts_at_exact_sample_after_nonzero_acquisition_time():
    camera, marker = Camera(), Marker()
    candidate = project((0., 0., .25), camera, marker)
    gate, dock = FrameGate(camera), Dock(camera, marker.side_m)
    first = gate.update(.06, [Frame(.06, .06, (candidate,))])
    assert dock.update(.06, first, False, (1., 0.))[1] < 0.
    at_deadline = gate.update(16.06, [Frame(16.06, 16.06, (candidate,))])
    assert dock.update(16.06, at_deadline, False, (1., 0.)) == (0., 0., "center_timeout")


def test_undock_rejects_early_stale_and_deadline_release():
    undock = Undock(.61)
    assert not undock.release(.15, .15)
    undock.authorize(.30)
    assert not undock.release(.31, .30)
    assert not undock.release(.65, .40)
    assert not undock.release(1.10, 1.10)
    assert undock.update(1.10, .61, True) == (0., 0., "virtual_release_timeout")
    assert not undock.release(1.11, 1.11)


def test_undock_waits_one_sample_and_stops_on_contact_reassertion():
    undock = Undock(.61)
    undock.authorize(.30)
    assert undock.release(.65, .64)
    assert undock.update(.65, .61, True)[:2] == (0., 0.)
    assert undock.update(.66, .61, False)[:2] == (0., 0.)
    assert undock.update(.67, .61, True)[:2] == (-.12, 0.)
    undock.contact = True
    assert undock.update(.68, .61, False) == (0., 0., "release_hold_lost")


def test_recovered_stream_discards_requests_until_authorized_in_new_epoch():
    stream = CommandInput(recovery=True)
    assert stream.update(0, 0., .01)[0].v_mps == .14
    for tick, event in ((5, "link_down"), (10, "link_restored"), (15, "guard_restart"),
                        (20, "fresh_authorization"), (25, "stale_epoch_request")):
        stream.handle_event(tick/100, event)
        command, sample = stream.update(tick, tick/100, .01)
        assert command.v_mps == 0. and not sample["stream_forwarded"]
    command, sample = stream.update(30, .30, .01)
    assert command.v_mps == .14 and sample["stream_forwarded"]
    assert sample["stream_input_epoch"] == sample["stream_epoch"] == 1


def test_operator_latch_distinguishes_delivery_from_guard_acceptance():
    stream = CommandInput()
    command, sample = stream.update(0, 0., .01)
    assert command.v_mps == .14 and sample["stream_accepted"]
    stream.handle_event(.05, "stream_operator_cancel")
    command, sample = stream.update(5, .05, .01)
    assert command.reason == "operator_abort" and command.v_mps == 0.
    assert sample["stream_forwarded"] and not sample["stream_accepted"]


def test_odometry_outage_expires_with_publisher_still_sending():
    stream = CommandInput()
    for tick in range(50):
        command, _ = stream.update(tick, tick/100, .01)
    assert command.v_mps > 0.
    stream.handle_event(.50, "odometry_stream_cut")
    for tick in range(50, 76):
        command, sample = stream.update(tick, tick/100, .01)
    assert command.reason == "odom_timeout" and command.v_mps == 0.
    assert sample["stream_odom_frame_stamp_s"] is None and sample["stream_odom_last_valid_stamp_s"] == .49
    assert sample["stream_forwarded"] and not sample["stream_accepted"]


def test_regressive_odometry_latches_zero_after_a_newer_sample_arrives():
    stream = CommandInput()
    for tick in range(50):
        command, _ = stream.update(tick, tick/100, .01)
    assert command.v_mps > 0.
    stream.handle_event(.50, "odometry_stamp_rewind")
    command, frame = stream.update(50, .50, .01)
    assert command.reason == "odom_regression" and command.v_mps == 0.
    assert frame["stream_odom_frame_stamp_s"] == .40 and frame["stream_odom_last_valid_stamp_s"] == .49
    command, frame = stream.update(55, .55, .01)
    assert command.reason == "odom_regression" and command.v_mps == 0.
    assert frame["stream_forwarded"] and not frame["stream_accepted"]
