"""Görseller gerçek kayıtları ve fiziksel geometrinin birimlerini kullanmalı."""
import csv
import json
import re
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from caster_physics.parameters import load_parameters
from caster_physics.visualization import create_animation, create_plots


@pytest.fixture
def reference_results():
    root = Path(__file__).resolve().parents[1]
    parameters = load_parameters()
    results = {}
    for name in ('aligned_forward', 'misaligned_forward', 'passive_coast'):
        with (root/'reference/matlab_m2_v1/results'/f'{name}_states.csv').open() as stream:
            records = [{key: float(value) for key, value in row.items()} for row in csv.DictReader(stream)]
        results[name] = SimpleNamespace(records=records, parameters=parameters,
            column=lambda key, rows=records: np.array([row[key] for row in rows]))
    return results


def test_comparison_plots_use_reference_records(reference_results, tmp_path):
    files = create_plots(reference_results, tmp_path)
    assert [path.name for path in files] == ['trajectory_comparison.png', 'angles_and_energy.png']
    assert all(path.read_bytes().startswith(b'\x89PNG') for path in files)
    import matplotlib.pyplot as plt
    assert plt.get_fignums() == []


def test_animation_embeds_actual_samples_geometry_and_offline_controls(reference_results, tmp_path):
    destination = tmp_path/'robot_motion.html'
    assert create_animation(reference_results, destination) == destination
    html = destination.read_text()
    assert 'https://' not in html and '<script src=' not in html
    assert 'id="play"' in html and 'id="restart"' in html and 'id="scrub"' in html
    assert 'requestAnimationFrame' in html and 'let playing = false' in html
    assert 'Sentetik' in html and 'COM' in html and 'phi' in html and 'theta' in html
    payload = json.loads(re.search(r'<script id="simulation-data" type="application/json">(.*?)</script>', html, re.S)[1])
    assert [case['name'] for case in payload['cases']] == ['aligned_forward', 'misaligned_forward']
    case = payload['cases'][1]
    reference = reference_results['misaligned_forward']
    assert case['samples'][0]['phi'] == reference.records[0]['caster_angle_body_rad']
    assert case['samples'][-1]['axle'] == [reference.records[-1]['x_m'], reference.records[-1]['y_m']]
    assert case['geometry']['drive_track_m'] == reference.parameters.geometry.drive_track_m
    for sample in case['samples']:
        assert np.all(np.isfinite([sample['time'], sample['theta'], sample['phi'], *sample['axle'], *sample['com'], *sample['pivot'], *sample['caster']]))
        assert np.linalg.norm(np.array(sample['pivot'])-sample['caster']) == pytest.approx(reference.parameters.geometry.caster_trail_m)


def test_single_case_and_empty_input(reference_results, tmp_path):
    single = {'passive_coast': reference_results['passive_coast']}
    assert create_animation(single, tmp_path/'single.html').exists()
    with pytest.raises(ValueError, match='deney'):
        create_animation({}, tmp_path/'empty.html')
    with pytest.raises(ValueError, match='deney'):
        create_plots({}, tmp_path)
