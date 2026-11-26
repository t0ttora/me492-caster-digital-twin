"""Çıktı dosyası bir bağlantı olsa bile başka kaynak dosyaya yazılmamalı."""
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from caster_physics.parameters import load_parameters
from caster_physics.result_files import save_results, write_json
from caster_physics.verification import read_csv
from caster_physics.visualization import create_plots, create_animation


@pytest.mark.parametrize("kind", ["csv", "json", "png", "html"])
def test_linked_output_never_overwrites_another_file(tmp_path, kind):
    target = tmp_path/"pretend_matlab_source.m"
    target.write_text("original MATLAB source")
    output = tmp_path/"python-output"
    output.mkdir()
    names = {"csv": "aligned_forward_states.csv", "json": "report.json",
             "png": "trajectory_comparison.png", "html": "robot_motion.html"}
    (output/names[kind]).symlink_to(target)
    reference = Path(__file__).parents[1]/"reference/matlab_m2_v1/results/aligned_forward_states.csv"
    rows = read_csv(reference)[:2]
    result = SimpleNamespace(records=rows, parameters=load_parameters(), step_s=.0005,
                             column=lambda name: np.array([row[name] for row in rows]))
    results = {"aligned_forward": result}
    with pytest.raises(ValueError, match="bağlantı|symlink"):
        if kind == "csv":
            save_results(results, output)
        elif kind == "json":
            write_json(output/names[kind], {"test": True})
        elif kind == "png":
            create_plots(results, output)
        else:
            create_animation(results, output/names[kind])
    assert target.read_text() == "original MATLAB source"
