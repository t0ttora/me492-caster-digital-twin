# JANUS motion and QR docking experiments

The JANUS case register covers 12 motion, 20 QR and 8 operational cases.
The Python V2.3 extension uses the existing caster model and adds
waypoint gates, camera-relative QR observations and explicit docking
events. The ROS 2/Gazebo package includes simulation sources and tests
that can run offline on Mac. The Windows Simscape runs use a separate
native model copy.

[Technical report in Turkish](../../reports/Arche_JANUS_Teknik_Rapor_TR.pdf) ·
[40-case analysis table](../../reports/janus/case-analysis.tsv)

## Python motion and docking

The S path measured 12.62 mm cross-track RMSE. The clockwise and
counterclockwise 0.50 m squares each reached eight gates, one for each
edge and each turn. The square gates completed at 32.50 s in both
directions. The signed QR correction turned toward the observed marker,
then waited 0.40 s in the inner band before driving forward. Missing,
incorrect and stale QR inputs were rejected. Alignment, contact and
charge were checked as separate simulated events. The Python bundle
keeps failed cases alongside their revised variants. The reviewed set
contains 36 passes, 11 partial results and 3 failed gates across 50
synthetic run variants. Nineteen targeted regressions passed after
repairing cancel, contact dwell, BMS hold and QR ambiguity handling.

[Python source and reproduction](https://github.com/t0ttora/me492-caster-digital-twin/tree/main/caster-physics-python-v2) ·
[S path motion](../../reports/janus/python/M04-python-nominal/motion.html) ·
[Clockwise square motion](../../reports/janus/python/M05-python-cw/motion.html) ·
[Left QR approach motion](../../reports/janus/python/Q02-python-left/motion.html) ·
[QR loss motion](../../reports/janus/python/Q06-python-nominal/motion.html) ·
[Python metrics CSV](../../reports/janus/python/measurements.csv)

## ROS 2 and Gazebo

Mac offline results cover 20 tests, 27 fixture-only checks and 162
integrated checks before review. The revised source passed 32 tests,
30 fixture checks and 39 checks on changed Python inputs. Three
offline recovery fixtures require a new command after recovery. The
integrated runs included actual QR decoding on
synthetic PNGs, timestamped message replays and open-loop velocity
integration. The warm detector p95 on the Mac was 12.1–33.6 ms across
fixtures. This timing measures preloaded images and excludes Gazebo
rendering, ROS transport and the full robot loop. Gazebo and ROS 2
runtime, bag capture and applied effort await a Linux run.

[ROS 2 and Gazebo source and runbook](https://github.com/t0ttora/me492-caster-digital-twin/tree/main/janus/ros2) ·
[Offline case table](../../reports/janus/ros2/case_results.csv) ·
[Ideal replay reference errors](../../reports/janus/ros2/tracking-proxy.csv)

## Native baseline and model comparison

The nine-run Simscape caster/step matrix completed on Elo in MATLAB
R2026b. The +30° 25 µs case reproduced the archived alignment trace at
its 100 µs samples. The tested 50, 25 and 12.5 µs results did not meet
the full-window swivel-rate convergence threshold of 0.0001 rad/s.
The largest deltas occur during the initial contact transient.
All six adjacent pairs also exceeded the gate in the final 0.5 s.
The additional +30° 6.25 µs run still differed from 12.5 µs by
0.002633 rad/s at the maximum common-grid sample.

The Python equal-torque comparison reached 3.39 mm forward displacement
at 3 s, alongside 68.85 mm in the November 16 native archive. The
lower-order Python plant and 3D native contact solve use different
contact laws. Compare their same-input responses as a numerical
diagnostic and measure the real robot separately.

[Simscape baselines figure](../../reports/janus/native-baselines.png) ·
[Native terminal values](../../reports/janus/native-baseline-terminal-values.csv) ·
[Step comparison CSV](../../reports/janus/native-baseline-comparisons.csv) ·
[Equal-torque Python comparison](../research/janus-torque-comparison.md)

## Native measured-pose and equal-effort checks

The native S candidate using the two 0.30 m arcs and +45° caster
reached 1.07 mm geometric cross-track RMSE and finished 11.25 mm
from the terminal pose target. Its measured terminal criteria held
after the first verified dwell. This run uses its own sensed pose
and its own controller. The nominal Python S variant used the same
arc geometry with a different speed profile and controller. Both
native square directions passed all four edge and four turn dwells.
The revised clockwise terminal hold finished 11.12 mm from its
target. The first clockwise candidate remains documented with one
of eight gates.

The four V4 QR runs use the projected physical marker center. All
four reached the first geometric gate and finished just beyond its
265 mm depth limit. In the finite blackout, the first invalid sample
and zero forward request were logged at 1.01 s. The sampled stop dwell
completed at 1.33 s, with 1.53 mm of travel to the start of the verified
stop. The observation returned and the run resumed. These cases use
an ideal camera projection. Dock contact and BMS confirmation need
separate measurements.

Eight open-loop Simscape runs replayed verified Python axle torque
streams. All eight matched both applied channels on the logged
native grids. When the independent Python review changed two QR
effort streams, those two native replays were repeated as J02.
Six earlier traces stayed byte-identical. In the nominal S replay,
position RMSE between the
plants was 1.24 m. The equal effort and matched horizontal startup
state establish the input comparison. The plants still have
different contact, support and friction formulations.

[Native Simscape code and models](https://github.com/t0ttora/me492-caster-digital-twin/tree/main/models/simscape-janus) ·
[Native S acceptance data](../../reports/janus/native-m04-acceptance.json) ·
[V4 QR loss force and command traces](../../reports/janus/native-qr-loss-v4.png) ·
[Open-loop equal-effort figure](../../reports/janus/native-equal-torque.png) ·
[Latest equal-effort measurements](../../reports/janus/native-equal-torque-reviewed.csv) ·
[Native square gate checks](../../reports/janus/native-m05-acceptance.json) ·
[6.25 µs solver comparison](../../reports/janus/native-refinement-6p25us.csv) ·
[Visual navigation shortlist](../research/janus-navigation-policy.md)

The M12 feasibility run used a 3° infinite plane. It recorded
axle height, pitch and per-wheel contact. A finite threshold needs
an edge and additional contact geometry. A GUI Scope capture or
Mechanics Explorer recording is unavailable in the batch evidence.
The figures use native workspace exports.
[3° terrain figure](../../reports/janus/native-m12-slope.png)

## Physical measurements

The physical pilot needs synchronized raw QR approach video, commands,
encoder, IMU and BMS/charger logs, plus camera and QR mount geometry
and an independent pose reference. This release contains synthetic and
native-simulation experiments. Physical accuracy depends on fitting
and checking a distinct set of operator-assisted runs.
