# JANUS motion and QR docking experiments

The JANUS case register covers 12 motion, 20 QR and 8 operational cases.
The Python V2.3 extension uses the existing caster model and adds
waypoint gates, camera-relative QR observations and explicit docking
events. The ROS 2/Gazebo package has both Mac offline fixtures and a
native Linux runtime. The Windows Simscape runs use a separate native
model copy. All 40 cases met their scoped simulation checks in the
current qualification matrix. Physical commissioning, contact geometry
approval and charging validation are tracked with the operator-assisted
pilot.

[Technical report in Turkish](../../reports/Arche_JANUS_Teknik_Rapor_TR.pdf) ·
[40-case qualification matrix](../../reports/janus/qualification-matrix.tsv) ·
[Illustrated case cards in Turkish](../../reports/Arche_JANUS_Vaka_Kartlari_TR.pdf) ·
[First-run case archive](../../reports/janus/case-analysis.tsv)

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
synthetic run variants. Requalification separates stale observations,
odometry regression, command loss and explicit cancellation. The revised
camera-mount case derives its optical target from the declared mechanical
dock pose and measures final model error of 4.615 mm and 0.472°. This
uses a projected camera and virtual contact.

The final V2.3 regression suite passed 83 tests. The V1 baseline passed
54 tests in its separate verification run.

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
rendering, ROS transport and the full robot loop.

Ubuntu 24.04 on arm64 ran ROS 2 Jazzy with Gazebo Harmonic. Four project
packages built. Live tests recorded OGRE2 RGB, decoded QR identity,
simulation clock, odometry, independent world pose, joints and applied
guarded velocity. The M02 braking fixture reached the stop band
63 ms after braking and held it for 250 ms. In Q07 the relocated marker
required 0.406 s of renewed centering, and measured forward motion followed.
One Q13 MCAP recorded 26,095 messages and reported transport drops.
The case-specific proof comes from the joined raw trace.
The final ROS suite passed 37 tests on Linux. The Mac offline suite
passed 36 and skipped the optional ZBar test. Seven mission-contract
regressions passed separately.

In Q08, the live decoder read 47 wrong-station observations. The lab
controller reached its 3.200 s simulation deadline with
`search_timeout:wrong_id`. Forward and success counts were both zero.
In O05, the Python plant and Linux Gazebo independently held stop while
fresh odometry returned and unauthorized requests were rejected. An
explicit authorization and a separate new request preceded measured
motion in both environments.

Q10 uses Gazebo-rendered RGB with separately labeled glare derivatives.
The live plant stopped from motion under hotspot, reflection and whiteout
stimuli. Q14 and O03 used scripted hazard inputs with independent world
pose and contact records. Q14 stopped within 2.681 mm. O03 stopped within
6.081 mm and its withheld-permission control stayed stopped after clearance.
Q04 passed its optical-versus-mechanical distinction check. Gazebo reached
261.58 mm optical standoff in the comparison run, with 43.70 mm and 0.0866
rad nominal mechanical error. A separate four-corner staging experiment
on the same world and camera fixture measured 6.236 mm mechanical error,
258.93 mm optical depth and a 280 ms terminal hold. That experiment uses
the fixture's declared marker yaw and an independently logged world pose.
Contact and charging remain separate pilot measurements.

[ROS 2 and Gazebo source and runbook](https://github.com/t0ttora/me492-caster-digital-twin/tree/main/janus/ros2) ·
[Live telemetry and acceptance files](../research/janus-ros-live-data.md) ·
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

M09's native short launch, in-place yaw command and later position
trim finished within the accepted pose band for both initial caster
angles. In a separate sequence with zero forward request throughout,
the 90° caster case reached the pose band. The best 180° result across
three monotone profiles and an opposite-yaw prefix was 26.78 mm from the
start, above the 20 mm bound. Both protocols appear in the case card.

The first four V4 QR runs used the projected marker center. All four
finished just beyond the 265 mm depth limit. Revised centered, left and
right runs held the depth and 20 mm pose bounds together for 0.40 s.
Their native plant produces measured pose and torque. The ideal camera
projects numerical corners for the controller. The mirrored r5 variants
reached their optical gate at 115.61 s, with positive forward requests kept inside
4% on the independently reprojected 1 ms grid. The earlier r4 variant
was faster and had brief excursions on the same grid. In the finite
blackout, the first invalid sample
and zero forward request were logged at 1.01 s. The sampled stop dwell
completed at 1.33 s, with 1.53 mm of travel to the start of the verified
stop. The observation returned and the run resumed. These cases use
an ideal camera projection. Dock contact and BMS confirmation need
separate measurements.

In a separate persistent-loss run, the invalid frame was captured
at 1.050 s. The command fell to zero at 1.060 s, the 250 ms stop dwell
completed at 1.417 s, and the test-only one-second loss timeout latched
an abort at 2.060 s. Commands stayed at zero until the 18 s run ended.
The matched-gain centered QR comparison measured a 5.04 nm maximum
position difference between the two center definitions in that symmetric
fixture. [Supplemental QR measurements](../../reports/janus/native-qr-supplements.json)

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

M12 compares the 3° native plane and dedicated finite fixtures. Gazebo
measured a full 5 mm traversal, with 87.06 mm minimum body clearance and
594.31 N peak caster normal force. Its 20 mm sensitivity test stalled.
The native model completed a separately identified 5 mm raised trim
with 1° beveled entry and exit. All wheels made measured trim contact.
The derived lower bound on chassis clearance was 74.07 mm. The native
sharp 5 mm curb and 3° uphill tests stalled or reversed. The figures use
native workspace exports.
[3° terrain figure](../../reports/janus/native-m12-slope.png)

## Physical measurements

The physical pilot needs synchronized raw QR approach video, commands,
encoder, IMU and BMS/charger logs, plus camera and QR mount geometry
and an independent pose reference. This release contains synthetic and
native-simulation experiments. Physical accuracy depends on fitting
and checking a distinct set of operator-assisted runs.
