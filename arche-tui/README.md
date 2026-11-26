# Arche Caster Lab TUI

Start by double-clicking `Start_Arche_TUI.command` in the project root, or run:

    ./Start_Arche_TUI.command

Uses the existing V1 Python environment, NumPy and Matplotlib. No new dependencies.
The interface and help text are English for presentation.

## Presentation flow

Press **1**, then **Enter** to run V1. Press **2**, then **Enter** to run V2. Wait for **COMPLETE**. Press **Enter** to inspect caster motion, **Enter** again for the results, and **Enter** once more to return to setup.

On setup, press the letter printed beside an input to edit it. For V2, A chooses the path, B edits the distance and C edits the segment time. Type a replacement value and press Enter to apply it, then Enter to run. V1 uses A for the experiment and B for duration; custom torque mode displays its torque fields instead. Dropdowns accept the printed numbers 1 through 9 directly, or arrows and Enter. Esc cancels an edit.

Each model retains its own edited inputs during the session. Enter runs those inputs. **W** resets the selected model to its original inputs and starts a default run. The starred model in the left menu is selected for the next run. Data marked RECORDED still belong to the saved run named above the page.

## Direct keys

| Key | Action |
| --- | --- |
| 1 / 2 | Select V1 / V2 and return to setup |
| 3 | Setup |
| 4 | Robot dimensions |
| 5 | Mass and friction |
| 6 | Starting state |
| 7 | Controller gains |
| 8 | Data and metrics |
| 9 | Saved runs |
| 0 | How to use |
| A-F on input pages | Edit the input with that letter |
| N / P on input pages | Next / previous six fields |
| I | All experiment inputs |
| R | Run with current inputs or stop an active run |
| X | Stop an active run |
| G | Four graphs |
| A / B / C / D on graphs | Caster angles / XY motion / wheel speeds / energy |
| C / E on a detailed plot | Choose signal / export PNG |
| T / Y / C on results | Table / summary / choose signal |
| Enter on saved runs | Open the row selected with arrows |
| C on saved runs | Compare selected row with the opened run |
| O | Browser viewer; on Saved runs, open the selected row |
| V / L on input pages | Save / load a configuration file |
| Esc | Previous page |
| Q | Quit when idle |

The footer states the actions available on the current page. Letters inside an edit dialog belong to that dialog; numeric input never switches models. Tab and arrow navigation remain available as an alternative to direct keys. F1-F5 also open help, setup, graphs, results and run/stop. Left/Right and F6 change focus for this alternative navigation.

## Guide and outputs

Press **0** for the separate English guide. Enter or N advances its pages; P goes back. Use a fullscreen terminal for larger plots. An 80 x 24 terminal also works.

A live run shows the last 600 recorded samples. COMPLETE loads the full saved dataset; check metrics separately to see whether tracking tolerance was reached. Stopping a run keeps its partial live trace; a stopped run may have no complete CSV.

Each completed run saves its inputs, effective parameters, results.csv, summary.json and run.log in a new folder under arche-tui/outputs. Previous results remain saved. Browser viewer opens an offline HTML explorer with channels, recorded-sample playback and the first 500 numeric rows; complete data remain in CSV.

## What the models mean

**V1:** the original torque-driven planar mechanical baseline. Choose one of the 12 existing experiments, or constant left/right axle torque. Presets load their original starting conditions and may deliberately disable resistance/contact terms. The effective parameter file records those ablations.

**V2:** the same mechanical kernel with its separate corrected-inertia parameter set, reference generation, wheel PI, tracking and terminal pose recovery. Select straight/arc/reversal/S-turn and baseline/quintic timing. Geometry/surface presets apply their fields immediately; later manual edits win. The comparison labels are not hardware measurements.

**Controller auto mode:** the existing design equations calculate gains from current inertia, command period and straight-reference speed. Manual gain edits switch to manual mode. The implementation's existing saturation and completion logic stay fixed: +/-0.5 N m wheel torque, 0.25 m/s speed, 1 rad/s yaw, 6/12 mm position-mode hysteresis; completion requires 2 cm, 0.05 rad, 0.02 m/s, 0.05 rad/s for 0.25 s. Gains are nominal design choices.

Changing radius directly does not silently change inertia. The named V2 radius presets use the original fixed-mass radius-squared scaling. Extreme parameters or large solver steps can fail validation or integration; review the stored log and config.

## Outputs and preservation

Each run saves request/config JSON, effective parameters, `results.csv`, `summary.json` and `run.log`. Exports and config snapshots also go under `arche-tui/outputs/`. Existing experiment outputs and the V1/V2 source/parameter files are never overwritten by the UI. The adapter uses existing model functions. V1 preservation hashes are checked before each run. Plot playback uses recorded samples, not a second simulator.

This is a synthetic planar model, not a calibrated physical robot, 3D contact engine or ROS 2 node. Previous CSVs are read without changing them. No new ROS, CAD or MATLAB integration is required for the presentation.

## Check

    ./caster-physics-python/.venv/bin/python -m unittest discover -s arche-tui -p 'test_*.py'
