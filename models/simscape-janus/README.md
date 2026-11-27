# JANUS native Simscape experiments

The 27 November 2026 baseline was executed on Windows in MATLAB R2026b
with Simulink, Simscape and Simscape Multibody. Nine three-second runs
crossed +30°, −30° and 0° initial caster angle with 50, 25 and 12.5 µs
fixed-step ode4. Both axle inputs were 0.01 N m. The candidate models
add measured axle-midpoint pose, per-contact normal-force and penetration
exports and effort signals to the preserved V6 replay. The native SLX
files are kept by run and solver step.

The +30° 25 µs run agrees with the 16 November reconstructed archive
on its 100 µs grid in the reported position, yaw, caster angle and
swivel-rate signals. Full-window refinement comparisons exceeded the
0.0001 rad/s swivel-rate criterion. The plant runs on a synthetic flat
surface and the study has no real-robot trials.

[Terminal values](../../reports/janus/native-baseline-terminal-values.csv) ·
[Step-size comparisons](../../reports/janus/native-baseline-comparisons.csv) ·
[Baseline figure](../../reports/janus/native-baselines.png)

## Reproduce the nine-case batch

Stage the baseline scripts on the MATLAB host. Put the preserved V6
archive beside the batch directory in the layout used by the executed
native scripts. The commands below run from the repository root in a
PowerShell session where MATLAB is on PATH.

```powershell
$work = Join-Path $PWD 'janus-native-rerun'
$batch = Join-Path $work 'baselines'
$source = Join-Path $work 'evidence_export_20261116_013532/latest_alignment_replay'
New-Item -ItemType Directory -Path $batch,$source | Out-Null
Copy-Item models/simscape-janus/baselines/*.m $batch
Copy-Item models/simscape-v6/evidence_latest_alignment_replay.slx $source
Copy-Item models/simscape-v6/evidence/full_replay.csv $source
matlab -wait -batch "cd('$batch'); run_baselines; analyze_baselines"
```

Reproduce in a new directory so existing results and source models remain
available. Initial, transient and settled windows answer different questions.
Use the same logged signal and sampled time grid for both step sizes.

## Replay the same recorded torque

Each `replay/<run-id>/` directory holds a native model with the
verified zero-order-held command matrix in its model workspace.
The adjacent (command, startup-state and scenario) files identify
the matrix contents and the Python run. A clean MATLAB R2026b
session can open the saved candidate and run it directly.

To rebuild the models from source, stage `replay-tools/*.m` as the
batch directory using the same neighboring V6 archive layout above.
Stage a `mac-inputs/J01/<run-id>/` folder inside that batch for each
replay directory. Copy its `scenario.json`, `manifest.json` and
`commands.csv`, naming the last one `applied_torque.csv`. Run
`matlab -wait -batch "cd('$batch'); run_mac_replay"` from a shell
where `$batch` points to the replay batch. The public input manifest
keeps the startup/command fields and records the original private
manifest hash. It omits machine-specific reproduction paths.

The applied channels matched all eight input traces on the logged
native grids. The measured poses diverged on several cases.
Compare the [new replay table](../../reports/janus/native-equal-torque.csv)
and [trajectory plot](../../reports/janus/native-equal-torque.png).
A native open-loop playback contains no QR controller or route
feedback; its pose divergence is a model-response result.

## Run the measured-pose candidates

The `closed-loop/<scenario>/` folders pair the saved native SLX and
its Level-2 MATLAB S-function. Copy the desired folder to the
simulation host, set MATLAB's current directory to that folder, then
open and run the model. [Published model index](published-models.json)
records content hashes and each candidate's internal gate flags.
Re-run in a copy. Internal flags and independently checked terminal
hold are reported separately in the JANUS release evidence.

QR candidates project a synthetic marker from the native plant's
measured axle pose. Use the `*_v4` directories for the projected
physical center, which is equivalent to the intersection of the two
projected diagonals. Earlier candidates used the corner mean. The
native fixtures use their own nominal camera mount and station
geometry. The logged visibility, image alignment and geometric
range gates cover the synthetic camera. Dock contact and BMS need
separate inputs.

The supplemental `q01_mean_production` model switches the center
calculation to the arithmetic mean while retaining the reference's
wheel gains. The `q06prime_persistent_abort` fixture uses a 1.0 s
continuous loss timeout and holds the abort through run end.
[Supplemental QR results](../../reports/janus/native-qr-supplements.json)

## Terrain feasibility

The 3° infinite-plane run contains a tilted plane, axle height and
pitch, per-wheel normal forces and penetration. Its three-second
trajectory moved about 73.6 mm downslope. It has no finite edge or
height discontinuity, so it does not evaluate traversing a threshold.
[Terrain figure](../../reports/janus/native-m12-slope.png)

The 6.25 µs refinement model and retained comparison are separate
from the original nine-run baseline matrix. A full solver-rate
6.25 µs run left the 0.0001 rad/s swivel-rate gate open.
[6.25 µs comparison](../../reports/janus/native-refinement-6p25us.csv)
