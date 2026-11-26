"""Regression coverage for signed geometry, interlocks and independent route gates."""
from dataclasses import replace
from math import pi
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from qr_camera import Camera, DOCK_ID, Frame, Marker, estimate_marker, project
from qr_docking import Dock, FrameGate
from scenarios import Dwell, FIXTURES, Route, terminal_ok
from twin import load_parameters


@pytest.mark.parametrize("yaw,sign", [(-.27, -1), (.27, 1)])
def test_projected_image_and_steering_have_opposite_signs(yaw, sign):
    camera, marker = Camera(), Marker()
    gate = FrameGate(camera)
    candidate = project([0., 0., yaw], camera, marker)
    assert candidate.visible
    assert [candidate.corners_px[i][1] < candidate.corners_px[i+2][1] for i in (0, 1)] == [True, True]
    obs = gate.update(0., [Frame(0., 0., (candidate,))])
    v, w, _ = Dock(camera, marker.side_m).update(0., obs, True, (1., yaw))
    assert obs.error*sign > 0.
    assert v == 0.
    assert w*sign < 0.
    distance, orientation = estimate_marker(candidate, camera, marker.side_m)
    assert distance > .8
    assert orientation == pytest.approx(-yaw)


def test_gate_only_accepts_a_single_expected_id_and_rejects_clock_faults():
    camera, marker = Camera(), Marker()
    gate = FrameGate(camera)
    good = project([0.,0.,0.],camera,marker)
    other = replace(good, id="JANUS-OTHER-99")
    assert gate.update(0., [Frame(0.,0.,(other,good))]).candidate.id == DOCK_ID
    assert gate.update(.05,[Frame(0.,.05,(good,))]).reason == "disordered"
    assert gate.update(.10,[Frame(.10,.10,(good,good))]).reason == "ambiguous_id"
    assert gate.update(.20,[Frame(.20,.20,())]).reason == "missing"
    assert gate.update(.25,[Frame(.25,.25,(other,))]).reason == "wrong_id"
    assert gate.update(.30,[Frame(.30,.30,(good,))]).valid
    assert gate.update(.50,[]).valid
    assert gate.update(.51,[]).reason == "stale"
    assert gate.update(.55,[Frame(.55,.54,(good,))]).reason == "invalid_clock"


def test_valid_center_dwell_then_loss_recenter_and_outer_band():
    camera, marker = Camera(), Marker()
    gate = FrameGate(camera)
    dock = Dock(camera, marker.side_m)
    centered = project([0.,0.,0.],camera,marker)

    def advance(t, candidates=(), stopped=False):
        obs = gate.update(t, [Frame(t,t,candidates)])
        return dock.update(t, obs, stopped, (0.,0.))

    for t in (0., .1, .2, .3, .39):
        assert advance(t, (centered,))[0] == 0.
    assert advance(.4,(centered,))[0] > 0.
    assert advance(.45)[0] == 0.
    assert dock.retries == 1
    for t in (.5, .6, .7, .8, .89):
        assert advance(t,(centered,))[0] == 0.
    assert advance(.9,(centered,))[0] > 0.
    uncentered = project([0.,0.,.25],camera,marker)
    assert advance(.95,(uncentered,))[:2] == (0.,0.)
    assert dock.phase == "recenter"


def test_virtual_bms_needs_full_stop_dwell_contact_and_fresh_timestamp():
    camera, marker = Camera(), Marker()
    gate = FrameGate(camera)
    dock = Dock(camera, marker.side_m)
    candidate = project([.61,0.,0.], camera,marker)
    for t in (0., .10, .20, .30, .40):
        obs = gate.update(t, [Frame(t,t,(candidate,))])
        dock.update(t,obs,False,(0.,0.),bms_time_s=t)
    assert dock.phase == "stop"
    assert not dock.contact and not dock.charge_confirmed
    for t, stopped in ((.50,True),(.70,False),(.71,True),(.95,True)):
        dock.update(t,obs,stopped,(0.,0.),bms_time_s=t)
        assert not dock.contact and not dock.charge_confirmed
    dock.update(.96,obs,True,(0.,0.))
    assert dock.aligned and dock.contact and not dock.charge_confirmed
    dock.update(1.00,obs,True,(0.,0.),bms_time_s=.96)
    assert not dock.charge_confirmed
    dock.update(1.30,obs,True,(0.,0.),bms_time_s=.96)
    assert not dock.charge_confirmed
    dock.update(1.32,obs,True,(0.,0.),bms_time_s=1.30)
    assert dock.charge_confirmed and dock.phase == "success"


@pytest.mark.parametrize("case,sign", [("M05",-1),("M06",1)])
def test_square_implements_all_gates_without_resetting_route_origin(case, sign):
    fixture = next(f for f in FIXTURES if f.case==case)
    route = Route(fixture, load_parameters(), np.zeros(3))
    t = 0.
    for i, target in enumerate(route.origins[1:]):
        assert route.segment == i
        t += route.durations[i]
        route.update(t,target,.021,0.,0.)
        route.update(t+.24,target,0.,0.,0.)
        route.update(t+.30,target,0.,0.,.051)
        route.update(t+.31,target,0.,0.,0.)
        route.update(t+.55,target,0.,0.,0.)
        assert len(route.gates) == i
        t += .56
        route.update(t,target,0.,0.,0.)
        assert route.gates[-1]["dwell_start_s"] == pytest.approx(t-.25)
        assert route.gates[-1]["kind"] == ("edge" if i%2==0 else "turn")
    assert route.done
    assert route.origins[-1] == pytest.approx([0.,0.,sign*2*pi])
    assert not terminal_ok((.021,0.,0.,0.))
    assert not terminal_ok((0.,.051,0.,0.))


def test_dwell_resets_on_invalid_sample():
    gate = Dwell(.25)
    assert not gate.update(0.,True)
    assert not gate.update(.24,False)
    assert not gate.update(.25,True)
    assert not gate.update(.49,True)
    assert gate.update(.50,True)


def test_invalid_future_frame_and_corners_do_not_block_next_valid_capture():
    camera, marker = Camera(), Marker()
    gate = FrameGate(camera)
    good = project([0.,0.,0.],camera,marker)
    assert gate.update(0.,[Frame(5.,5.,(good,))]).reason == "invalid_clock"
    bad = replace(good, corners_px=((320.,240.),)*4)
    assert gate.update(.05,[Frame(.05,.05,(bad,))]).reason == "invalid_corners"
    assert gate.update(.10,[Frame(.10,.10,(good,))]).valid


def test_valid_but_off_center_frames_cannot_command_yaw_indefinitely():
    camera, marker = Camera(), Marker()
    gate = FrameGate(camera)
    dock = Dock(camera, marker.side_m)
    candidate = project([0.,0.,.25],camera,marker)
    for t in (0., 1., 15.99):
        obs = gate.update(t,[Frame(t,t,(candidate,))])
        v, w, _ = dock.update(t,obs,False,(1.,1.))
        assert v == 0. and w < 0.
    obs = gate.update(16.,[Frame(16.,16.,(candidate,))])
    assert dock.update(16.,obs,False,(1.,1.)) == (0.,0.,"center_timeout")
    assert dock.phase == "abort"
