"""Bağımsız fizik beklentileri; MATLAB kodunun satır satır kopya testi değil."""
from dataclasses import replace

import numpy as np
import pytest

from caster_physics.state import State, Torques
from caster_physics.physics.motion import evaluate_motion
from caster_physics.physics.inertia import mass_matrix_and_bias
from caster_physics.physics.support_loads import support_loads
from caster_physics.physics.geometry import geometry_at


def test_rest_is_an_equilibrium(parameters):
    evaluation = evaluate_motion(State.at_rest(parameters), Torques(0, 0), parameters)
    np.testing.assert_allclose(evaluation.derivative, 0, atol=1e-12)
    assert evaluation.dissipation_power_W == 0


def test_weight_and_moments_balance_for_every_caster_angle(parameters):
    for angle in np.linspace(-np.pi, np.pi, 73):
        state = replace(State.at_rest(parameters), caster_angle_relative_rad=angle)
        normal = support_loads(state, parameters)
        geometry = geometry_at(state, parameters)
        g, d = parameters.geometry, parameters.dynamics
        caster_from_axle = np.array(g.caster_pivot_m[:2]) - g.caster_trail_m * np.array([np.cos(angle), np.sin(angle)])
        assert min(normal) > 5
        assert sum(normal) == pytest.approx((d.body_lumped_mass_kg+d.caster_lumped_mass_kg)*d.gravity_mps2)
        assert normal[2]*caster_from_axle[0] == pytest.approx(d.gravity_mps2*(d.body_lumped_mass_kg*d.body_com_m[0]+d.caster_lumped_mass_kg*caster_from_axle[0]))
        assert g.drive_track_m/2*(normal[0]-normal[1])+normal[2]*caster_from_axle[1] == pytest.approx(d.gravity_mps2*(d.body_lumped_mass_kg*d.body_com_m[1]+d.caster_lumped_mass_kg*caster_from_axle[1]), abs=1e-12)
        matrix, _ = mass_matrix_and_bias(state, parameters, geometry)
        np.testing.assert_allclose(matrix, matrix.T, atol=1e-14)
        assert np.linalg.eigvalsh(matrix).min() > 1e-5


def test_spinning_unpowered_drive_wheel_pushes_forward_and_slows(parameters):
    state = replace(State.at_rest(parameters), left_wheel_rate_radps=100)
    evaluation = evaluate_motion(state, Torques(0, 0), parameters)
    assert evaluation.contacts.left_force_world_N[0] > 0
    assert evaluation.derivative[8] < 0


def test_lateral_friction_opposes_sliding(parameters):
    state = replace(State.at_rest(parameters), body_velocity_y_mps=0.2)
    evaluation = evaluate_motion(state, Torques(0, 0), parameters)
    assert evaluation.contacts.left_force_world_N[1] < 0
    assert evaluation.dissipation_power_W >= 0


def test_caster_ablation_keeps_bearing_mass_and_support(parameters):
    state = replace(State.at_rest(parameters), caster_relative_rate_radps=0.8, body_velocity_x_mps=0.2)
    on = evaluate_motion(state, Torques(.12, -.03), parameters)
    off = evaluate_motion(state, Torques(.12, -.03), parameters, caster_contact_enabled=False)
    np.testing.assert_array_equal(off.contacts.caster_force_world_N, [0, 0])
    assert off.contacts.caster_rolling_torque_Nm == off.contacts.bore_torque_Nm == 0
    assert off.contacts.bearing_torque_Nm == on.contacts.bearing_torque_Nm != 0
    np.testing.assert_array_equal(off.normal_loads_N, on.normal_loads_N)


def test_instantaneous_energy_balance_and_shared_friction_budget(parameters):
    state = State.from_vector(np.array([-.08, 0, .3, .7, .2, -.1, .4, -.8, 4, 2, 3, 0, 0]))
    torques = Torques(.12, -.03)
    evaluation = evaluate_motion(state, torques, parameters)
    delta = 1e-6
    plus = State.from_vector(state.as_vector()+delta*evaluation.derivative)
    minus = State.from_vector(state.as_vector()-delta*evaluation.derivative)
    # Enerji, kütle matrisinden bağımsız olarak nokta hızlarıyla hesaplanır.
    from caster_physics.physics.energy import kinetic_energy
    derivative = (kinetic_energy(plus, parameters)-kinetic_energy(minus, parameters))/(2*delta)
    assert derivative == pytest.approx(evaluation.motor_power_W-evaluation.dissipation_power_W, abs=1e-7)
    contacts = evaluation.contacts
    budget = np.linalg.norm([*contacts.caster_force_world_N, contacts.bore_torque_Nm/parameters.dynamics.caster_bore_effective_radius_m])
    assert budget <= parameters.dynamics.caster_friction_coefficient*evaluation.normal_loads_N[2]+1e-10


def test_symmetry_and_caster_effect(run_case):
    aligned, positive, negative = [run_case(name) for name in ("aligned_forward", "misaligned_forward", "mirror_misaligned")]
    np.testing.assert_allclose(aligned.column("yaw_rad"), 0, atol=1e-10)
    np.testing.assert_allclose(positive.column("yaw_rad"), -negative.column("yaw_rad"), atol=1e-8)
    np.testing.assert_allclose(positive.column("y_m"), -negative.column("y_m"), atol=1e-8)
    assert np.max(np.abs(positive.column("yaw_rad"))) > .001


def test_passivity_conservation_and_step_refinement(run_case):
    coast = run_case("passive_coast")
    assert np.max(np.diff(coast.column("kinetic_energy_J"))) < 1e-9
    assert coast.column("kinetic_energy_J")[-1] < .1*coast.column("kinetic_energy_J")[0]
    free = run_case("conservative_free")
    for key in ["kinetic_energy_J", "linear_momentum_x_kgmps", "linear_momentum_y_kgmps", "angular_momentum_z_kgm2ps"]:
        values = free.column(key)
        assert np.max(np.abs(values-values[0])) < 1e-8
    coarse, fine, finer = [run_case(name) for name in ("misaligned_forward", "misaligned_fine", "misaligned_finer")]
    coarse_error = np.max(np.abs(coarse.column("yaw_rad")-fine.column("yaw_rad")))
    fine_error = np.max(np.abs(fine.column("yaw_rad")-finer.column("yaw_rad")))
    assert coarse_error < 1e-4
    assert fine_error <= max(.3*coarse_error, 1e-10)
