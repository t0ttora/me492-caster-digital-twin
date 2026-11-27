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
sys.path.insert(0, "arche-tui")
from backend import defaults, simulate_config, validate

def launch(settings_json):
    settings = json.loads(settings_json)
    config = defaults(settings["model"])
    config["initial"]["caster_angle_relative_rad"] = math.radians(settings["caster"])
    if settings["model"] == "V2":
        config["experiment"].update(maneuver=settings["maneuver"], profile=settings["profile"])
    params, duration = validate(config)
    rows, _, _ = simulate_config(config, params, duration)
    data = io.StringIO()
    writer = csv.DictWriter(data, fieldnames=rows[0].keys())
    writer.writeheader()
    writer.writerows(rows)
    samples = [[row["time_s"], row["x_m"], row["y_m"],
                math.degrees(row["caster_angle_body_rad"])] for row in rows]
    return json.dumps({
        "samples": samples,
        "csv": data.getvalue(),
        "duration": rows[-1]["time_s"],
        "peakTorque": max(abs(row[key]) for row in rows
                           for key in ("left_drive_torque_Nm", "right_drive_torque_Nm"))
    }, allow_nan=False)
`);
  return instance;
}

self.onmessage = async ({data}) => {
  try {
    python ??= await initialize();
    self.postMessage({status: 'Integrating the simulation…'});
    const run = python.globals.get('launch');
    try {
      self.postMessage({result: JSON.parse(run(JSON.stringify(data)))});
    } finally {
      run.destroy();
    }
  } catch (error) {
    self.postMessage({error: error.message});
  }
};
