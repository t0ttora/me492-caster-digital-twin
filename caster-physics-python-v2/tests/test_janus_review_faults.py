"""Fault injections at the JANUS plant, frame gate and dock controller interfaces."""
from dataclasses import replace
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from janus_metrics import summarize
from janus_sim import simulate
from qr_camera import Camera, Frame, Marker, project
from qr_docking import Dock, FrameGate
from scenarios import FIXTURES, Fixture
from twin import Segment


def fixture(case, variant="nominal"):
    return next(f for f in FIXTURES if (f.case, f.variant) == (case, variant))


@pytest.mark.parametrize(("case", "variant", "events"), [
    ("O07", "low-soc", ((0., "transit"), (.10, "operator_cancel"), (.20, "low_soc"))),
    ("O07", "low-soc", ((0., "transit"), (.10, "operator_cancel"))),
    ("M02", "nominal", ((.10, "operator_cancel"),)),
    ("O03", "resume", ((.10, "operator_cancel"), (.15, "authorized_resume"),
                       (.20, "odometry_timeout"))),
])
def test_cancel_preempts_every_transit_path(case, variant, events):
    trial = replace(fixture(case, variant), events=events, duration_s=1.5)
    run = simulate(trial)
    rows = run["rows"]
    assert any(row["requested_v_mps"] > 0. for row in rows if row["time_s"] < .10)
    assert all((row["requested_v_mps"], row["requested_yaw_rate_radps"], row["phase"], row["reason"])
               == (0., 0., "abort", "operator_cancel") for row in rows if row["time_s"] >= .10)
    assert run["completed_s"] is not None
    assert run["termination_reason"] == "operator_cancel"
    assert not run["gates"]
    if trial.segments:
        metrics = summarize(trial, run)
        assert metrics["stop_trigger_time_s"] == .10
        assert metrics["checks"]["terminal_gate"] is False
        assert summarize(trial, {**run, "termination_reason": "route_gates_accepted"})["checks"]["terminal_gate"] is False


def test_camera_transit_honors_explicit_brake():
    trial = replace(fixture("O07", "low-soc"), events=((0., "transit"), (.10, "brake_step"),
                                                      (.20, "low_soc")), duration_s=1.5)
    run = simulate(trial)
    assert run["termination_reason"] == "brake_step"
    assert run["completed_s"] is not None
    assert all((row["requested_v_mps"], row["requested_yaw_rate_radps"]) == (0., 0.)
               for row in run["rows"] if row["time_s"] >= .10)


def test_cancel_beats_already_accepted_route_gate_and_survives_short_log():
    trial = Fixture("M03", segments=(Segment(0., 0., .10),),
                    events=((.40, "operator_cancel"),), duration_s=.42)
    run = simulate(trial)
    assert [g["gate"] for g in run["gates"]] == [1]
    assert run["completed_s"] is None
    assert run["termination_reason"] == "operator_cancel"
    assert summarize(trial, run)["checks"]["terminal_gate"] is False


def dock_at_standoff():
    camera, marker = Camera(), Marker()
    gate, dock = FrameGate(camera), Dock(camera, marker.side_m)
    candidate = project((.61, 0., 0.), camera, marker)
    for tick in range(9):
        time_s = tick * .05
        obs = gate.update(time_s, [Frame(time_s, time_s, (candidate,))])
        dock.update(time_s, obs, False, (0., 0.))
    assert dock.phase == "stop"
    return dock, obs


def test_mechanical_dwell_restarts_on_position_yaw_or_speed_violation():
    dock, obs = dock_at_standoff()
    for tick in range(41, 103):
        t = tick / 100
        position = .021 if tick < 66 else 0.
        yaw = math.radians(3.1) if tick == 72 else 0.
        dock.update(t, obs, tick != 77, (position, yaw))
        assert not dock.contact and not dock.aligned
    dock.update(1.03, obs, True, (0., 0.))
    assert dock.aligned and dock.contact
    assert dock.phase == "wait_for_charge"


@pytest.mark.parametrize(("stopped", "pose_error"), [
    (True, (.021, 0.)),
    (True, (0., math.radians(3.1))),
    (False, (0., 0.)),
])
def test_lost_mechanical_hold_blocks_fresh_bms_and_invalidates_contact(stopped, pose_error):
    dock, obs = dock_at_standoff()
    for tick in range(41, 67):
        dock.update(tick / 100, obs, True, (0., 0.))
    assert dock.phase == "wait_for_charge"
    assert dock.contact
    assert dock.update(.67, obs, stopped, pose_error, bms_time_s=.67) == (0., 0., "mechanical_hold_lost")
    assert dock.phase == "abort"
    assert not dock.contact and not dock.aligned and not dock.charge_confirmed
    dock.update(.68, obs, True, (0., 0.), bms_time_s=.68)
    assert not dock.charge_confirmed


def test_lost_contact_before_terminal_stop_revokes_provisional_charge():
    dock, obs = dock_at_standoff()
    for tick in range(41, 67):
        dock.update(tick / 100, obs, True, (0., 0.))
    dock.update(.67, obs, True, (0., 0.), bms_time_s=.67)
    assert dock.charge_confirmed
    assert dock.update(.68, obs, True, (.021, 0.)) == (0., 0., "mechanical_hold_lost")
    assert not dock.contact and not dock.charge_confirmed


def test_camera_error_matches_projection_of_the_physical_marker_center():
    camera, marker = Camera(), Marker()
    x, y, yaw = .61, -.0537, .12
    candidate = project((x, y, yaw), camera, marker)
    dx = marker.x_m - x
    dy = marker.y_m - y
    depth = math.cos(yaw)*dx + math.sin(yaw)*dy - camera.forward_m
    right = math.sin(yaw)*dx - math.cos(yaw)*dy
    expected_error = camera.fx_px*right / depth / camera.width_px
    obs = FrameGate(camera).update(0., [Frame(0., 0., (candidate,))])
    assert obs.valid
    assert obs.error == pytest.approx(expected_error, abs=1e-12)
    assert abs(obs.error) > Dock.INNER


@pytest.mark.parametrize("unknown_id", [None, ""])
def test_expected_target_plus_undecoded_quadrangle_never_advances(unknown_id):
    camera, marker = Camera(), Marker()
    good = project((0., 0., 0.), camera, marker)
    unreadable = replace(good, id=unknown_id)
    gate, dock = FrameGate(camera), Dock(camera, marker.side_m)
    seen = []
    for tick in range(11):
        t = tick / 20
        obs = gate.update(t, [Frame(t, t, (good, unreadable))])
        seen.append((obs.reason, dock.update(t, obs, True, (1., 0.))[:2]))
    assert seen == [("ambiguous_undecoded", (0., 0.))] * len(seen)
    assert gate.update(.55, [Frame(.55, .55, (unreadable,))]).reason == "undecoded"
    known = replace(good, id="KNOWN-DISTRACTOR")
    assert gate.update(.60, [Frame(.60, .60, (good, known))]).valid


def test_search_needs_fresh_empty_frames_and_q05_projection_emits_no_candidate():
    trial = replace(fixture("Q05", "acquire"), events=((.01, "stream_cut"),), duration_s=.30)
    run = simulate(trial)
    first = run["rows"][0]
    assert run["frames"][0].candidates == ()
    assert first["qr_reject_reason"] == "missing"
    assert first["requested_yaw_rate_radps"] > 0.
    assert all(row["requested_yaw_rate_radps"] == 0. and row["qr_reject_reason"] == "stale"
               for row in run["rows"] if row["time_s"] > .20)
    for events, reason in [(((0., "stream_cut"),), "no_frame"), (((0., "stale"),), "stale")]:
        rows = simulate(replace(trial, events=events))["rows"]
        assert rows[-1]["qr_reject_reason"] == reason
        assert all(row["requested_yaw_rate_radps"] == 0. for row in rows)
    no_search = simulate(Fixture("Q01", yaw_deg=55., duration_s=.10))
    assert all(row["requested_yaw_rate_radps"] == 0. for row in no_search["rows"])


def test_fresh_target_arriving_at_search_deadline_does_not_restart_motion():
    camera, marker = Camera(), Marker()
    dock, gate = Dock(camera, marker.side_m), FrameGate(camera)
    candidate = project((0., 0., 0.), camera, marker)
    obs = gate.update(3.2, [Frame(3.2, 3.2, (candidate,))])
    assert dock.update(3.2, obs, True, (1., 0.)) == (0., 0., "search_timeout:valid")
    assert dock.phase == "abort"


def test_explicit_sweep_ignores_a_held_empty_observation_after_expiry():
    camera, marker = Camera(), Marker()
    dock, gate = Dock(camera, marker.side_m, search_sweep=True), FrameGate(camera)
    obs = gate.update(0., [Frame(0., 0., ())])
    assert dock.update(0., obs, True, (1., 0.))[:2] == (0., -.55)
    assert dock.update(.21, obs, True, (1., 0.))[:2] == (0., 0.)


@pytest.mark.parametrize(("case", "variant"), [("O04", "watchdog"), ("O05", "clock-reset")])
def test_injected_stop_records_braking_scope_without_detector_acceptance(case, variant):
    trial = fixture(case, variant)
    run = simulate(trial)
    metrics = summarize(trial, run)
    assert trial.hardware_scope
    assert metrics["checks"]["injected_stop_trigger_observed"]
    assert metrics["checks"]["post_trigger_braking_dwell"]
    assert "fault_latched" not in metrics["checks"]


def test_odometry_reauthorization_requires_distinct_command_and_measured_restart():
    trial = fixture("O05", "guard-odometry-reauthorization")
    run = simulate(trial)
    result = summarize(trial, run)
    assert all(result["checks"].values())
    recovery = result["odometry_guard_reauthorization"]
    assert recovery["guard_zero_s"] < recovery["fresh_odometry_s"] < result["stop_dwell_verified_at_s"]
    assert len(recovery["fresh_unauthorized_rejected_request_stamps_s"]) >= 2
    assert recovery["authorization_s"] < recovery["new_command_event_s"] < recovery["restarted_forward_motion_above_stop_band_s"]


def test_odometry_restoration_and_separate_command_cannot_bypass_missing_authority():
    nominal = fixture("O05", "guard-odometry-reauthorization")
    trial = replace(nominal, events=tuple(entry for entry in nominal.events if entry[1] != "odometry_recovery_authorize"))
    run = simulate(trial)
    result = summarize(trial, run)
    assert result["checks"]["watchdog_zero_from_measured_motion"]
    assert result["checks"]["fresh_unauthorized_commands_rejected_by_guard"]
    assert not result["checks"]["authorization_alone_kept_zero"]
    assert not result["checks"]["only_distinct_new_command_restarted_measured_motion"]
    new = next(row for row in run["rows"] if row["stream_explicit_recovery_command"])
    assert new["stream_odom_frame_stamp_s"] == new["time_s"]
    assert new["stream_forwarded"] and not new["stream_accepted"]
    assert new["reason"] == "recovery_permission_required"
    assert all(row["requested_v_mps"] == 0. for row in run["rows"] if row["time_s"] >= new["time_s"])
