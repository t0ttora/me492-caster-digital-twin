"""Gain selection from explicit pole/time-constant requirements."""
from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import twin


def test_gains_place_the_selected_tracking_and_sampled_wheel_poles():
    design = getattr(twin, 'controller_gains', None)
    assert callable(design), 'Gains must be calculated from declared design requirements'
    parameters = twin.load_parameters()
    g = design(parameters)
    assert g['tracking_x'] == pytest.approx(2.)
    assert g['tracking_y'] == pytest.approx(400/9)
    assert g['tracking_heading'] == pytest.approx(2.)
    lateral = np.array([[0., .15], [-g['tracking_y']*.15, -g['tracking_heading']]])
    np.testing.assert_allclose(np.sort(np.linalg.eigvals(lateral)), [-1., -1.], atol=1e-7)
    J, h = parameters.dynamics.drive_roll_inertia_kgm2, parameters.dynamics.command_sample_dt_s
    a, b = h*g['wheel_p']/J, h*h*g['wheel_i']/J
    wheel = np.array([[1-a-b, -h/J], [g['wheel_i']*h, 1.]])
    np.testing.assert_allclose(np.sort(np.linalg.eigvals(wheel)), [.5, .8], atol=1e-12)
    assert g['wheel_p'] == pytest.approx(.0675)
    assert g['wheel_i'] == pytest.approx(1.125)


def test_controller_uses_the_calculated_pi_gains():
    p = twin.load_parameters()
    controller = twin.TrackingController(p, 'straight', 'baseline')
    torque = controller.wheel_torques(twin.State.at_rest(p), [2., 2.], .01)
    assert torque.left_Nm == pytest.approx(.1575)
    assert torque.right_Nm == pytest.approx(.1575)
