"""Behavior checks, independent mechanics identities, and strict replay contracts."""
import csv
from dataclasses import replace
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from twin import (MANEUVERS, PROFILES, DESIGNS, SURFACES, State, Torques, advance_pose,
                  case_parameters, geometry_at, integrate, load_parameters, pose_errors,
                  profile_duration, reference_at, run_case, verify_v1, wheel_odometry)
from caster_physics.physics.inertia import mass_matrix_and_bias
from caster_physics.physics.motion import evaluate_motion
from caster_physics.physics.energy import kinetic_energy
from evaluate_recording import evaluate
from run_study import new_output_directory


def test_v1_files_match_preserved_snapshot():
    assert verify_v1() == {"passed": True, "checked_files": 52}


@pytest.mark.parametrize("maneuver", MANEUVERS)
def test_profiles_have_the_same_path_end_and_peak_speed(maneuver):
    final = []
    for profile in PROFILES:
        duration = profile_duration(maneuver, profile)
        final.append(reference_at(duration, maneuver, profile)[0])
        speeds = np.array([reference_at(t, maneuver, profile)[1:] for t in np.linspace(0, duration, 4001)])
        if profile == "baseline":
            peak = np.max(abs(speeds), axis=0)
        else:
            np.testing.assert_allclose(np.max(abs(speeds), axis=0), peak, atol=1e-6)
            assert reference_at(0, maneuver, profile)[1:] == (0., 0.)
        assert reference_at(duration, maneuver, profile)[1:] == (0., 0.)
    np.testing.assert_allclose(final[0], final[1], atol=1e-14)
    assert profile_duration(maneuver, "smooth") == 1.875*profile_duration(maneuver, "baseline")


def test_closed_form_odometry_for_straight_turn_reverse_and_wrapped_angles():
    parameters = load_parameters()
    times = np.linspace(0, 2, 201)
    for left, right in [(2., 2.), (1., 3.), (-2., -2.), (-2., 2.)]:
        poses = wheel_odometry(times, np.full(201, left), np.full(201, right), parameters, [0, 0, .3])
        radius, track = parameters.geometry.drive_radius_m, parameters.geometry.drive_track_m
        expected = advance_pose([0, 0, .3], radius*(left+right), 2*radius*(right-left)/track)
        np.testing.assert_allclose(poses[-1], expected, atol=1e-13)
    assert pose_errors([[0, 0, np.pi], [0, 0, np.pi]], [[0, 0, -np.pi], [0, 0, -np.pi]])["yaw_rmse_rad"] < 1e-14


@pytest.mark.parametrize("times,left,right", [([0, 0], [1, 1], [1, 1]),
    ([0, .1], [1], [1, 1]), ([0, np.nan], [1, 1], [1, 1])])
def test_bad_odometry_inputs_fail(times, left, right):
    with pytest.raises(ValueError):
        wheel_odometry(times, left, right, load_parameters(), [0, 0, 0])


@pytest.mark.parametrize("name", ["../parameters", ".", "..", ""])
def test_outputs_cannot_escape_or_overwrite(name):
    with pytest.raises(ValueError):
        new_output_directory(name)


def test_new_run_directory_preserves_existing_outputs_and_rejects_links(tmp_path, monkeypatch):
    import run_study
    monkeypatch.setattr(run_study, "ROOT", tmp_path)
    output = new_output_directory("existing-run")
    (output / "keep.txt").write_text("retained")
    with pytest.raises(FileExistsError):
        new_output_directory("existing-run")
    assert (output / "keep.txt").read_text() == "retained"
    other_root = tmp_path / "another-root"
    other_root.mkdir()
    (other_root / "outputs").symlink_to(output, target_is_directory=True)
    monkeypatch.setattr(run_study, "ROOT", other_root)
    with pytest.raises(ValueError, match="symlink"):
        new_output_directory("new-run")


def test_all_sensitivity_parameters_are_supported_without_changing_nominal():
    original = load_parameters()
    for surface in SURFACES:
        for design in DESIGNS:
            parameters = case_parameters(surface, design)
            assert min(evaluate_motion(State.at_rest(parameters), Torques(0, 0), parameters).normal_loads_N) > 0
    assert load_parameters() == original


def test_bias_matches_independent_finite_difference_lagrange_and_energy_identity():
    """Differentiate T(q,v), rather than copying the implemented centripetal expression."""
    parameters = load_parameters()
    random = np.random.default_rng(492)
    for _ in range(40):
        vector = random.normal(size=13)
        vector[11:] = 0
        state = State.from_vector(vector)
        velocities = state.velocities()
        matrix, bias = mass_matrix_and_bias(state, parameters, geometry_at(state, parameters))
        derivatives = np.zeros((7, 7, 7))
        for coordinate in range(4):
            delta = np.zeros(13)
            delta[coordinate] = 1e-6
            plus, minus = State.from_vector(vector+delta), State.from_vector(vector-delta)
            derivatives[coordinate] = (mass_matrix_and_bias(plus, parameters, geometry_at(plus, parameters))[0]
                                       - mass_matrix_and_bias(minus, parameters, geometry_at(minus, parameters))[0]) / 2e-6
        independent_bias = (np.einsum("kij,j,k->i", derivatives, velocities, velocities)
                            - .5*np.einsum("ijk,j,k->i", derivatives, velocities, velocities))
        np.testing.assert_allclose(bias, independent_bias, atol=1e-9, rtol=1e-7)
        assert np.min(np.linalg.eigvalsh(matrix)) > 0
        assert kinetic_energy(state, parameters) == pytest.approx(.5*velocities @ matrix @ velocities, abs=1e-12)
        evaluation = evaluate_motion(state, Torques(.1, -.2), parameters)
        plus = State.from_vector(vector+1e-7*evaluation.derivative)
        minus = State.from_vector(vector-1e-7*evaluation.derivative)
        energy_derivative = (kinetic_energy(plus, parameters)-kinetic_energy(minus, parameters))/2e-7
        assert energy_derivative == pytest.approx(evaluation.motor_power_W-evaluation.dissipation_power_W, abs=2e-5)
        assert evaluation.dissipation_power_W >= 0


@pytest.fixture(scope="module")
def reversal():
    return run_case("reversal", "smooth", "reference_synthetic")


def test_closed_loop_reversal_is_finite_passive_and_torque_limited(reversal):
    records, metrics, _ = reversal
    assert np.all(np.isfinite([list(row.values()) for row in records]))
    assert any(row["command_speed_mps"] < 0 for row in records)
    assert metrics["energy_balance_max_J"] < 1e-5
    assert metrics["torque_peak_Nm"] <= .5
    assert metrics["dissipation_min_W"] >= -1e-10
    assert records[0]["x_m"] == 0


def test_v2_yaw_and_position_converge_when_step_is_halved():
    parameters = load_parameters()
    start = replace(State.at_rest(parameters), caster_angle_relative_rad=.7)
    outputs = []
    for scale in (1., .5, .25):
        refined = replace(parameters, dynamics=replace(parameters.dynamics, solver_step_s=scale*.0005))
        outputs.append(integrate(refined, .2, start, lambda t, state: Torques(.2, .1)))
    arrays = [np.array([[r[key] for key in ("x_m", "y_m", "yaw_rad")] for r in rows]) for rows in outputs]
    coarse, fine = np.max(abs(arrays[0]-arrays[1])), np.max(abs(arrays[1]-arrays[2]))
    assert coarse < 1e-6
    assert fine <= max(.3*coarse, 1e-11)


@pytest.fixture
def recording(tmp_path):
    parameters = load_parameters()
    start = replace(State.at_rest(parameters), caster_angle_relative_rad=.7)
    rows = integrate(parameters, .2, start, lambda t, state: Torques(.2 if t < .1 else -.1, .1))
    recording_rows = [{**{key: row[key] for key in ("time_s", "left_drive_torque_Nm", "right_drive_torque_Nm",
                                                 "left_wheel_rate_radps", "right_wheel_rate_radps")},
                       **{f"reference_{key}": row[key] for key in ("x_m", "y_m", "yaw_rad")}} for row in rows]
    csv_path, metadata_path = tmp_path / "run.csv", tmp_path / "metadata.json"
    with csv_path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=recording_rows[0])
        writer.writeheader()
        writer.writerows(recording_rows)
    metadata = {"run_id": "synthetic-fixture", "session_id": "test", "split": "development",
                "data_kind": "synthetic", "surface_id": "reference_synthetic", "robot_revision": "v1-synthetic",
                "pose_source": "synthetic_m2_fixture", "initial_state": start.as_vector().tolist(),
                "pose_frame": "drive_axle_midpoint_world_SI", "torque_units": "axle_Nm",
                "parameter_provenance": "V1 synthetic defaults", "torque_source": "synthetic program"}
    metadata_path.write_text(json.dumps(metadata))
    return csv_path, metadata_path


def test_torque_replay_matches_a_known_synthetic_recording(recording):
    report, rows = evaluate(*recording)
    assert report["m2_torque_replay_vs_reference"]["position_max_m"] < 1e-12
    assert report["m0_encoder_vs_reference"]["position_rmse_m"] > 0
    assert report["real_robot_validated"] is False
    assert report["data_kind"] == "synthetic"
    assert len(rows) == 21


@pytest.mark.parametrize("change", [{"pose_source": "wheel_odometry"}, {"data_kind": "unknown"},
                                  {"pose_frame": "body_com"}, {"torque_units": "volts"}])
def test_replay_rejects_invalid_provenance(recording, change):
    csv_path, metadata_path = recording
    metadata = json.loads(metadata_path.read_text())
    metadata.update(change)
    metadata_path.write_text(json.dumps(metadata))
    with pytest.raises(ValueError):
        evaluate(csv_path, metadata_path)


def test_replay_rejects_missing_samples(recording):
    csv_path, metadata_path = recording
    lines = csv_path.read_text().splitlines()
    csv_path.write_text("\n".join(lines[:4]+lines[5:])+"\n")
    with pytest.raises(ValueError, match="gaps"):
        evaluate(csv_path, metadata_path)
