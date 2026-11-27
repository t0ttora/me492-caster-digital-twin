const runtime = 'https://cdn.jsdelivr.net/pyodide/v0.29.4/full/';
let python;

async function initialize() {
  const {loadPyodide} = await import(runtime + 'pyodide.mjs');
  const instance = await loadPyodide({indexURL: runtime});
  const sourceURL = new URL('../downloads/arche-python-lab.zip', import.meta.url);
  sourceURL.search = new URL(import.meta.url).search;
  const [archive] = await Promise.all([fetch(sourceURL), instance.loadPackage('numpy')]);
  if (!archive.ok) throw new Error('Could not load the public model sources.');
  instance.unpackArchive(await archive.arrayBuffer(), 'zip');
  instance.runPython(`
import csv
import io
import json
import math
import sys

sys.path[:0] = ["arche-tui", "scripts"]
import backend
from browser_examples import torque_baseline, compare_s_turn

EXAMPLES = {"torque_baseline": torque_baseline, "compare_s_turn": compare_s_turn}

def launch(example):
    runs = EXAMPLES[example]()
    data = io.StringIO()
    writer = csv.DictWriter(data, fieldnames=["run", *runs[0][1][0]])
    writer.writeheader()
    series = []
    for name, rows, metrics in runs:
        writer.writerows({"run": name, **row} for row in rows)
        samples = [[row["time_s"], row["x_m"], row["y_m"],
                    math.degrees(row["caster_angle_body_rad"]),
                    row.get("ref_x_m"), row.get("ref_y_m")] for row in rows]
        series.append({
            "name": name, "samples": samples,
            "duration": rows[-1]["time_s"],
            "pathRmse": metrics.get("path_rmse_m"),
            "completion": metrics.get("completion_time_s"),
            "peakTorque": max(abs(row[key]) for row in rows
                               for key in ("left_drive_torque_Nm", "right_drive_torque_Nm")),
        })
    return json.dumps({"series": series, "csv": data.getvalue()}, allow_nan=False)
`);
  return instance;
}

self.onmessage = async ({data}) => {
  try {
    python ??= await initialize();
    self.postMessage({status: 'Running the Python example…'});
    const run = python.globals.get('launch');
    try {
      self.postMessage({result: JSON.parse(run(data))});
    } finally {
      run.destroy();
    }
  } catch (error) {
    self.postMessage({error: error.message});
  }
};
