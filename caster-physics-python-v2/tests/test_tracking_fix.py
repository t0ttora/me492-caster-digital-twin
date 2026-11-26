"""Regressions for the terminal lateral deadlock and friction-loaded P servo."""
from dataclasses import replace
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import twin


def controller(parameters, maneuver="straight", profile="baseline"):
    kind = getattr(twin, "TrackingController", None)
    assert callable(kind), "A stateful tracker with terminal recovery is required"
    return kind(parameters, maneuver, profile)


def test_terminal_lateral_error_is_not_an_uncommanded_equilibrium():
    parameters = twin.load_parameters()
    target = twin.reference_at(2., "straight", "baseline")[0]
    state = replace(twin.State.at_rest(parameters), body_x_m=target[0]+parameters.dynamics.body_com_m[0],
                    body_y_m=.03)
    tracking = controller(parameters)
    torques = tracking(2., state)
    assert max(abs(torques.left_Nm), abs(torques.right_Nm)) > .01


def test_pi_wheel_servo_removes_constant_load_offset_without_windup():
    parameters = twin.load_parameters()
    tracking = controller(parameters)
    method = getattr(tracking, "wheel_torques", None)
    assert callable(method), "Wheel PI servo is required"
    rate = np.zeros(2)
    dt, inertia, disturbance = .01, parameters.dynamics.drive_roll_inertia_kgm2, np.array([.08, .10])
    target = np.array([2., 2.])
    for index in range(400):
        state = replace(twin.State.at_rest(parameters), left_wheel_rate_radps=rate[0], right_wheel_rate_radps=rate[1])
        torques = method(state, target, dt)
        torque = np.array([torques.left_Nm, torques.right_Nm])
        assert max(abs(torque)) <= .5
        rate += dt*(torque-disturbance)/inertia
    np.testing.assert_allclose(rate, target, atol=1e-5)
    np.testing.assert_allclose(tracking.integral_torque_Nm, disturbance, atol=1e-5)


@pytest.mark.parametrize("surface", twin.SURFACES)
def test_arc_turn_completes_with_original_tolerances(surface):
    rows, metrics, _ = twin.run_case("arc_turn", "baseline", surface)
    assert metrics["completion_time_s"] is not None
    assert metrics["tracking"]["endpoint_error_m"] < .02
    assert metrics["energy_balance_max_J"] < 1e-5


def test_completion_hold_uses_elapsed_time_not_a_fixed_sample_count():
    function = getattr(twin, "completion_time", None)
    assert callable(function), "Completion hold must use timestamps and both planar velocity components"
    times = np.array([2., 2.05, 2.10, 2.15, 2.20, 2.25])
    rows = [{"time_s": t, "x_m": 0., "y_m": 0., "yaw_rad": 0., "body_long_velocity_mps": 0.,
             "body_lat_velocity_mps": 0., "body_yaw_rate_radps": 0.} for t in times]
    assert function(rows, np.zeros(3), 2.) == 2.
    rows[-1]["body_lat_velocity_mps"] = .1
    assert function(rows, np.zeros(3), 2.) is None


def test_tracking_command_obeys_the_lyapunov_identity_and_is_logged():
    parameters = twin.load_parameters()
    time = 1.
    target, vr, wr = twin.reference_at(time, "arc_turn", "baseline")
    state = replace(twin.State.at_rest(parameters), body_x_m=target[0]-.01+parameters.dynamics.body_com_m[0],
                    body_y_m=target[1]+.005, body_yaw_rad=target[2]-.1)
    tracking = controller(parameters, "arc_turn")
    tracking(time, state)
    assert hasattr(tracking, "commanded_speed_mps"), "Log the applied body commands for proof/code comparison"
    yaw = state.body_yaw_rad
    rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    axle = np.array([state.body_x_m, state.body_y_m])-rotation@np.array(parameters.dynamics.body_com_m[:2])
    ex, ey = rotation.T@(target[:2]-axle)
    et = twin.wrap(target[2]-yaw)
    v, w = tracking.commanded_speed_mps, tracking.commanded_yaw_radps
    ex_dot = w*ey-v+vr*np.cos(et)
    ey_dot = -w*ex+vr*np.sin(et)
    et_dot = wr-w
    derivative = ex*ex_dot+ey*ey_dot+np.sin(et)*et_dot/(400/9)
    assert derivative == pytest.approx(-2.*ex**2-.045*np.sin(et)**2, abs=1e-14)


def test_terminal_polar_command_has_the_claimed_error_decay():
    parameters = twin.load_parameters()
    # Goal (0.3,0), robot (0.27,-0.04), yaw zero: rho=.05, cos(alpha)=.6.
    state = replace(twin.State.at_rest(parameters), body_x_m=.19, body_y_m=-.04)
    tracking = controller(parameters)
    tracking(2., state)
    assert hasattr(tracking, "commanded_speed_mps"), "Expose the actual terminal commands"
    alpha, rho = np.arctan2(.04, .03), .05
    v, w = tracking.commanded_speed_mps, tracking.commanded_yaw_radps
    # This example saturates yaw, so verify the nominal law separately at small alpha.
    assert v == pytest.approx(2.*rho*np.cos(alpha))
    assert w <= 1.
    state = replace(state, body_y_m=-.001)
    tracking = controller(parameters)
    tracking(2., state)
    alpha, rho = np.arctan2(.001,.03), np.hypot(.03,.001)
    v, w = tracking.commanded_speed_mps, tracking.commanded_yaw_radps
    assert v/rho*np.sin(alpha)-w == pytest.approx(-3.*alpha, abs=1e-13)
    assert -v*np.cos(alpha) == pytest.approx(-2.*rho*np.cos(alpha)**2, abs=1e-13)
