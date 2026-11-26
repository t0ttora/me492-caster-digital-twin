"""Birim/çerçeve hataları ve ilk öğrenme akışının koruyucu sınırları."""
from dataclasses import replace
import json

import numpy as np
import pytest

from caster_physics.parameters import validate_parameters
from caster_physics.state import State, Torques
from caster_physics.physics.motion import evaluate_motion
from caster_physics.simulation.experiments import experiment_by_name, sampled_torques
from caster_physics.simulation.rk4 import simulate
from caster_physics.result_files import save_results, prepare_output_directory
from caster_physics.verification import verify_source_preservation, verify_reference_integrity, physical_run_checks


def test_rolling_without_slip_has_no_traction(parameters):
    speed = .2
    state = replace(State.at_rest(parameters), body_velocity_x_mps=speed,
                    left_wheel_rate_radps=speed/parameters.geometry.drive_radius_m,
                    right_wheel_rate_radps=speed/parameters.geometry.drive_radius_m,
                    caster_roll_rate_radps=speed/parameters.geometry.caster_radius_m)
    forces = evaluate_motion(state, Torques(0, 0), parameters).contacts
    np.testing.assert_allclose([forces.left_force_world_N, forces.right_force_world_N, forces.caster_force_world_N], 0, atol=1e-10)


def test_command_and_result_grids_are_distinct_and_zoh_is_preserved(parameters, tmp_path):
    experiment = replace(experiment_by_name("aligned_forward"), duration_s=.04)
    parameters = replace(parameters, dynamics=replace(parameters.dynamics, result_sample_dt_s=.02))
    result = simulate(parameters, experiment)
    np.testing.assert_allclose(result.column("time_s"), [0, .02, .04], atol=1e-15)
    np.testing.assert_allclose(result.column("left_drive_torque_Nm"), [0, .025, .05], atol=1e-15)
    assert result.states[0, 0] == -.08  # B COM
    assert result.records[0]["x_m"] == 0  # A aks merkezi
    assert result.records[-1]["x_m"] > 0
    save_results({experiment.name: result}, tmp_path)
    manifest = json.loads((tmp_path/"python_run_manifest.json").read_text())
    assert manifest["runs"][0]["rows"] == 3
    assert manifest["real_robot_validated"] is False
    assert physical_run_checks({experiment.name: result})["all_passed"]


def test_first_command_interval_has_zero_motion(parameters):
    experiment = replace(experiment_by_name("aligned_forward"), duration_s=.02)
    result = simulate(parameters, experiment)
    np.testing.assert_array_equal(result.states[1], result.states[0])
    assert result.column("left_drive_torque_Nm")[1] > 0
    assert result.states[2, 0] > result.states[1, 0]


@pytest.mark.parametrize("change", [
    {"body_lumped_mass_kg": -1}, {"solver_step_s": 0},
    {"drive_friction_coefficient": -.1}, {"result_sample_dt_s": .0003},
    {"body_com_m": (.8, 0, .12)}, {"bore_regularization_radps": float("nan")},
])
def test_invalid_physical_parameters_fail_loudly(parameters, change):
    with pytest.raises(ValueError):
        validate_parameters(replace(parameters, dynamics=replace(parameters.dynamics, **change)))


@pytest.mark.parametrize("value", [[0]*12, [0]*14, [float("nan")]*13, [[0]*13]])
def test_state_vector_contract_rejects_invalid_values(value):
    with pytest.raises(ValueError):
        State.from_vector(value)


def test_unknown_scenario_and_nonfinite_torque_are_rejected():
    with pytest.raises(ValueError):
        experiment_by_name("typo")
    with pytest.raises(ValueError):
        Torques(float("nan"), 0)


def test_bad_time_grids_are_rejected(parameters):
    with pytest.raises(ValueError):
        simulate(parameters, replace(experiment_by_name("zero_rest"), duration_s=0))
    with pytest.raises(ValueError):
        simulate(parameters, replace(experiment_by_name("zero_rest"), duration_s=.0101))


def test_output_protection_covers_source_directories():
    from caster_physics.parameters import PROJECT_ROOT
    for name in ["src", "parameters", "reference", "tests", "docs", "verification", ".venv"]:
        with pytest.raises(ValueError):
            prepare_output_directory(PROJECT_ROOT/name)
    with pytest.raises(ValueError):
        prepare_output_directory(PROJECT_ROOT.parent/"ME492_Public_Tracker")


def test_reference_and_original_sources_have_integrity():
    assert verify_source_preservation()["all_passed"]
    assert verify_reference_integrity()["all_passed"]


def test_reversal_program_switches_after_independent_ramp_schedule(parameters):
    experiment = experiment_by_name("reversal_perturbed")
    assert sampled_torques(100, parameters, experiment).left_Nm == .06
    assert sampled_torques(150, parameters, experiment).left_Nm == -.25
