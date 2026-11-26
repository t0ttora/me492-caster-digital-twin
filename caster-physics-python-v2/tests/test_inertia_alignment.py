"""Wheel-only spin inertias and the handwritten isolated-wheel experiment."""
from dataclasses import replace
from pathlib import Path
import sys
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from twin import load_parameters, case_parameters, State, Torques, integrate


def test_wheel_spin_inertias_match_the_declared_uniform_cylinders():
    p = load_parameters()
    assert p.dynamics.drive_roll_inertia_kgm2 == pytest.approx(.001125)
    assert p.dynamics.caster_roll_inertia_kgm2 == pytest.approx(.000320)
    assert p.dynamics.drive_roll_inertia_kgm2 == pytest.approx(.5*.4*p.geometry.drive_radius_m**2)
    assert p.dynamics.caster_roll_inertia_kgm2 == pytest.approx(.5*.4*p.geometry.caster_radius_m**2)


def test_isolated_drive_wheel_matches_hand_calculation():
    p = load_parameters()
    p = replace(p, dynamics=replace(p.dynamics, drive_friction_coefficient=0.,
                drive_rolling_resistance_coefficient=0., drive_roll_viscous_NmspRad=0.))
    rows = integrate(p, .2, State.at_rest(p), lambda t, s: Torques(.01, 0.))
    assert rows[-1]['left_wheel_rate_radps'] == pytest.approx(1.7777777777777777, abs=1e-12)
    np.testing.assert_allclose([[r['x_m'], r['y_m'], r['yaw_rad']] for r in rows], 0, atol=1e-12)


@pytest.mark.parametrize('design', ['radius_30mm','radius_50mm'])
def test_radius_sweep_updates_wheel_spin_inertia(design):
    p = case_parameters('reference_synthetic', design)
    assert p.dynamics.caster_roll_inertia_kgm2 == pytest.approx(.5*.4*p.geometry.caster_radius_m**2)
