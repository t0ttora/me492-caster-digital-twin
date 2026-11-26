# Pre-visit roadmap: Onshape, MATLAB and Gazebo

Decision date: Wednesday 7 October 2026. Preparation deadline: **Friday 9 October 2026, 11:00 Europe/Istanbul**. Working baseline locked by the student. Demonstration freeze: Friday 09:15.

## The decision

Use **Onshape** for the editable mechanical assembly, **MATLAB** for the primary caster model and analysis, and **Gazebo** for a small robot-integration demonstration. Prepare a provisional model before the company visit, then replace assumptions with company dimensions and measurements. The CAD and simulation work can overlap; completing a detailed CAD replica is not a prerequisite for the first MATLAB calculation.

The Friday deliverable is a preparation demonstrator, not a calibrated Arche digital twin. No real robot experiments have been established. Semester research scope and G1-G6 dates remain unchanged.

## What must be ready

| Priority | Deliverable | Acceptance evidence |
| --- | --- | --- |
| 1 | Five-body parametric Onshape assembly | Changing drive-wheel radius, track width, caster pivot position or trail updates the assembly without breaking its joints. Two drive-wheel rotations, caster swivel and caster roll remain distinct. Save its URL and a named version. |
| 2 | Small MATLAB baseline | Run straight motion, reversal and a stationary turn; log body pose and caster angle. Repeat with different initial caster angles. Record software release, parameters and commands. Claim execution only when a run completes. |
| 3 | Minimal Gazebo scene | Two drive wheels and one actual swivel/rolling caster on a flat floor. Command forward/reverse and a turn; confirm motion and support stability. Record startup and run evidence. |
| 4 | Meeting pack | A parameter/assumption sheet, required measurements, short demonstration recordings where available, the literature PDF and a prioritised company request list. |

MATLAB and Gazebo runs can be independent. Reuse the same named dimensions, coordinate convention and scenario definitions; identical numerical outputs are not a Friday acceptance requirement. Each closed-loop experiment later needs an explicit choice of which environment owns the robot dynamics.

## Verified starting conditions

At 18:27 on 7 October, this Mac had no MATLAB app in the standard Applications listing, and `matlab`, `gz`, `ros2`, `docker` and `octave` were absent from PATH. These checks apply only to this Mac. Onshape document access was subsequently verified in the signed-in browser.

The coordinated Windows chat **Değerlendir caster digital twin yol** freshly verified MATLAB R2026b, Simulink, Simscape and Simscape Multibody. A disposable Simulink unit-rate integrator ran to 1 s and returned position 1 (exit code 0; capability evidence timestamp 2026-10-07T16:22:39Z). This establishes execution access, not successful execution of the robot model. Gazebo/ROS installation on the dual-boot Ubuntu environment is student-reported; exact versions and runtime remain unverified.

The literature review and source-inspected MATLAB examples are already available locally. Their existence does not prove that this robot model has been run. The first work block resolves execution access before committing to a live Friday demonstration.

## Work blocks

These are proposed working appointments, not an assumption of continuous availability. They allocate **9 hours 45 minutes**, including 45 minutes of recovery buffer; this is a planning allocation, not a hard ceiling on personal learning time. If availability is lower, drop Gazebo detail before reducing the CAD and MATLAB baseline.

| Date and time | Work | Completion check / time limit |
| --- | --- | --- |
| Wed 7 Oct, 18:30-19:00 | Choose the execution computer. Open Onshape and verify document access. On the chosen computer, check MATLAB release, Simulink/Simscape Multibody availability and Gazebo availability. | Record the host and successful application startup or the exact blocker. End the check after 30 minutes; do not spend the evening on an uncertain installation. |
| Wed, 19:00-20:30 | Build the five-body Onshape model with explicit dimension variables and simple solids. Establish the drive-axle origin and caster pivot/axle axes. | Change one dimension, regenerate, and check the four rotating coordinates. Label every assumed dimension provisional. No shell, fastener or cosmetic modelling. |
| Wed, 20:30-21:00 | Save a CAD version and the parameter sheet; attempt the first export on an available runtime. | Inspect body count, scale and joint axes. Keep exported files and the source version reference. Unsuccessful export remains pending. |
| Thu 8 Oct, 09:30-11:30 | Build/run the small MATLAB motion baseline using the parameter sheet. Start with ideal caster kinematics and commands; add the imported assembly only if its joints and contacts are manageable. | Successful straight, reversal and turn runs; body pose and caster angle recorded. Compare aligned versus misaligned starts. No friction identification or accuracy claim from synthetic data. |
| Thu, 11:30-12:00 | Check the Onshape-to-Simscape route on the small assembly if the runtime is available. | Confirm scale, orientation, roll/swivel freedom and floating-base/contact needs. After 30 minutes without a clean import, retain the MATLAB baseline and record the import blocker. |
| Thu, 13:00-14:30 | Prepare the minimum Gazebo floor/robot scene on an available host. Use simple collision geometry and the same named dimensions. | Robot starts without falling/exploding; forward/reverse and turn commands produce motion. The caster must have separate swivel and roll joints, rather than a fixed sphere. Limit this block to 90 minutes. |
| Thu, 14:30-15:30 | Run the prepared scenarios, correct units/signs and preserve evidence. | Each successful run has its parameters, initial state and software/host recorded. Save plots and short recordings where useful. Record failure honestly if an environment remains unavailable. |
| Thu, 15:30-16:15 | Assemble the company question list and short demonstration narrative. | Identify which provisional values the company must replace and what minimal log/experiment would allow calibration. Prepare an offline demonstration route. |
| Thu, 16:15-17:00 | Recovery buffer for one demonstrated blocker. | Prefer repair of the primary MATLAB baseline. Do not start a new simulator, controller or contact formulation. |
| Fri 9 Oct, 08:30-09:15 | Reopen the saved artifacts and rehearse the demonstration. | Confirm access to saved CAD, parameters, successful run evidence and the research PDF. Freeze at 09:15; label any unexecuted work as prepared/pending. |
| Fri, 09:15-11:00 | Meeting/travel preparation reserve. | No new features or installations. Adapt this reserve to actual travel requirements; the preparation deadline remains 11:00. |

## Model boundaries and parameter ownership

The provisional CAD contains a chassis, two driven wheels, a fork and a caster wheel. Wheels/fork stay separate bodies. Use simple shapes and a floor-compatible support height. Keep wheel radius, drive track, caster pivot x/y, trail and caster wheel radius editable. Keep geometry and initial caster angle distinct.

The parameter sheet records name, meaning, unit, value, provenance and uncertainty/status. Before company evidence, values are explicitly synthetic or literature-derived. Mass and inertia from a simplified CAD are not accepted as the real robot's mass properties. A successful export is a connectivity/geometric check; ground contact, bearing resistance, actuation and parameter calibration remain separate work.

### Reconciled modelling contract (7 October)

- Build our own readable Simulink model. Published models are engineering references; copying a complete stock model is not the project deliverable.
- **M0:** ideal differential-drive motion with caster kinematics. Caster angle follows prescribed body motion; changing its initial angle must not alter the M0 body path. This is a numerical/frame check, not evidence that caster forces affect yaw.
- **M1:** drive response calibrated from robot data. **M2:** the same drive response plus caster force effects. Compare held-out measurements before claiming a prediction improvement.
- The Windows and Ubuntu installations are dual-boot. Use sequential replay of common SI parameters and command CSVs before attempting simultaneous coupling. One environment owns the dynamics in each experiment.
- Canonical parameters originate in the Windows starter's `outputs/caster_model_v0/parameters_si.json`; the [published parameter file](https://github.com/t0ttora/me492-caster-digital-twin/blob/main/models/caster-m0/parameters_si.json) records the same schema. Origin is the drive-axle midpoint projected to ground; x forward, y left, z up. The mirror is manually synchronised, not a live CAD/MATLAB link. Record CAD edits in the common file before replay.
- Baseline values are wholly synthetic: chassis 0.50 × 0.32 × 0.08 m, centre z 0.14 m; drive radius 0.075 m, width 0.03 m, track 0.38 m; caster radius 0.04 m, width 0.02 m, trail 0.05 m, pivot (-0.18, 0, 0.10) m.

Use one straight-start case with an aligned caster, repeat it with 90-degree and 180-degree initial orientations, then run a reversal and a stationary turn. Exact speed, duration and geometry are stored with each synthetic run. Physical execution limits are requested from the company before any real test.

## Transfer routes and fallback decisions

- **Onshape to Simscape:** use the documented `smexportonshape` / `smimport` route when the installed MATLAB release and account access allow it. First test the small assembly. If import consumes its allotted block, keep a parameterised MATLAB baseline; CAD import is not a prerequisite for numerical kinematics.
- **Onshape to Gazebo:** attempt assembly URDF export where available, then inspect the resulting bodies/joints. A minimal manually specified robot description is acceptable if export is unsuitable. Ground contact and drive control still require separate setup.
- **No MATLAB execution access:** preserve CAD, parameter sheet, export evidence and the prepared calculation/scenarios. State that MATLAB execution is pending and seek a verified licensed host. Do not substitute another engine and report it as MATLAB execution.
- **No Gazebo execution access or stable scene after its block:** freeze the robot description and the blocker. Demonstrate CAD and successful MATLAB work first. Gazebo remains a prepared integration task, not an executed result.
- **Neither runtime available:** Friday's honest outcome is the parametric CAD, documented model/scenarios and evidence requests. Do not claim a working simulation.

Sources for the transfer paths: [MathWorks Onshape exporter](https://www.mathworks.com/help/sm/ref/smexportonshape.html), [Onshape export formats](https://cad.onshape.com/help/Content/File/exporting_files.htm). These support a possible workflow, not verified execution on the current machines.

## What to request at the visit

1. Actual robot revision, driven-wheel/caster dimensions and pivot placement; existing CAD or measured drawings if available.
2. Mass, payload configuration and known mass properties; caster part number and construction.
3. Command semantics/limits and one timestamped sample log, preferably command, wheel feedback and IMU, with independent pose and caster-angle evidence where possible.
4. Operator-led access for a small repeatable experiment if source code cannot be shared. Request an owner and delivery/test dates.

Explain the demonstrator as: “We prepared a configurable model. These dimensions and measurements will let us adapt it to your robot and test its predictions.” Calibration and independent validation follow the visit; they are not due on Friday morning.

## Current status

- [x] Onshape / MATLAB-primary / minimal-Gazebo decision recorded.
- [x] Deadline, work blocks, acceptance checks and fallback rules recorded.
- [x] Limited local runtime availability checked; missing PATH tools recorded.
- [x] Windows MATLAB/Simulink/Simscape/Multibody execution access verified with a disposable run.
- [x] Parametric CAD starter created: five bodies/four mates, measured drive diameter and trail regeneration/swivel check; wider parameter ranges remain to be qualified.
- [x] Student-owned M0 Simulink baseline executed on Windows: six scenarios, 32/32 numerical checks, CSVs/plots and checked ZIP saved.
- [ ] Gazebo scene run and evidence saved.
- [ ] Friday demonstration pack frozen.

This is a local planning update. It does not publish files, book the visit, install software or establish physical-robot validation.

The [academic progress report](../../reports/Arche_Academic_Report_Typeset_EN_2026-11-16.pdf) records the concept CAD and modelling evidence. Gazebo and CAD-to-Multibody import/contact execution remain pending.
