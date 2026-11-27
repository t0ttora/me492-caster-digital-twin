"""Sample-boundary and interlock regressions for the requalification fixtures."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from janus_command_input import CommandInput
from janus_sim import CameraStream
from janus_undock import Undock
from qr_camera import Camera, Frame, Marker, project
from qr_docking import Dock, FrameGate


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
