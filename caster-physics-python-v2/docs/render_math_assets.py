"""Render the report's equations with the existing scientific Python environment."""
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
from matplotlib import mathtext, rcParams
from matplotlib.font_manager import FontProperties

rcParams["mathtext.fontset"] = "stix"
specification, output = Path(sys.argv[1]), Path(sys.argv[2])
output.mkdir(parents=True, exist_ok=True)
for index, expression in enumerate(json.loads(specification.read_text())):
    path = output / f"eq-{index:03d}.png"
    mathtext.math_to_image("$" + expression + "$", path, dpi=240, prop=FontProperties(size=13), color="#152838")
    assert path.stat().st_size > 100, path
