"""Kabul eşiği port yazılmadan belirlendi: 1e-7 mutlak + 1e-6 bağıl."""
import csv
import json
from pathlib import Path

import numpy as np
import pytest

from caster_physics.simulation.experiments import EXPERIMENTS

REFERENCE = Path(__file__).parents[1] / "reference/matlab_m2_v1"


@pytest.mark.parametrize("name", ["aligned_forward", "misaligned_forward", "passive_coast"])
def test_every_recorded_column_matches_matlab_over_time(name, run_case):
    result = run_case(name)
    with (REFERENCE / "results" / f"{name}_states.csv").open() as file:
        reader = csv.DictReader(file)
        names = reader.fieldnames
        expected = np.array([[float(row[key]) for key in names] for row in reader])
    actual = np.array([[row[key] for key in names] for row in result.records])
    assert result.records[0].keys() == dict.fromkeys(names).keys()
    np.testing.assert_allclose(actual, expected, atol=1e-7, rtol=1e-6)
    np.testing.assert_allclose(actual[:, :3], expected[:, :3], atol=1e-12, rtol=0)


@pytest.mark.parametrize("experiment", EXPERIMENTS, ids=lambda case: case.name)
def test_all_twelve_endpoints_match_native_matlab_manifest(experiment, run_case):
    reference = json.loads((REFERENCE / "run_manifest.json").read_text())
    expected = next(run for run in reference["runs"] if run["scenario_id"] == experiment.name)
    result = run_case(experiment.name)
    end = result.records[-1]
    np.testing.assert_allclose(
        [end["x_m"], end["y_m"], end["yaw_rad"]], expected["final_pose"], atol=1e-7, rtol=1e-6
    )
    np.testing.assert_allclose(end["caster_angle_body_rad"], expected["final_caster_angle_body_rad"], atol=1e-7, rtol=1e-6)
    np.testing.assert_array_equal(result.states[0], expected["initial_state"])


def test_original_matlab_files_are_unchanged():
    import hashlib
    project = Path(__file__).parents[1]
    manifest = json.loads((project / "verification/matlab_sources_before.json").read_text())
    for name, digest in manifest["files"].items():
        assert hashlib.sha256((project.parent / name).read_bytes()).hexdigest() == digest, name
