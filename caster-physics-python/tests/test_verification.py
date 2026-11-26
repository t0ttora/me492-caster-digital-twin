import csv
from pathlib import Path
from types import SimpleNamespace

from caster_physics.verification import compare_results

REFERENCE = Path(__file__).parents[1]/"reference/matlab_m2_v1"


def test_verifier_detects_a_corrupted_trajectory():
    with (REFERENCE/"results/aligned_forward_states.csv").open() as file:
        rows = [{key: float(value) for key, value in row.items()} for row in csv.DictReader(file)]
    rows[10]["x_m"] += .01
    report = compare_results({"aligned_forward": SimpleNamespace(records=rows)}, REFERENCE)
    assert not report["all_passed"]
    assert not report["full_series"]["aligned_forward"]["columns"]["x_m"]["passed"]


def test_verifier_rejects_missing_required_columns():
    with (REFERENCE/"results/aligned_forward_states.csv").open() as file:
        rows = [{key: float(value) for key, value in row.items()} for row in csv.DictReader(file)]
    del rows[0]["yaw_rad"]
    report = compare_results({"aligned_forward": SimpleNamespace(records=rows)}, REFERENCE)
    assert not report["all_passed"]
