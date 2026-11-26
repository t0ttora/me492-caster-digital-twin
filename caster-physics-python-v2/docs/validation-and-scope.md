# V1 verification and V2 research scope

**V2.1 update, 13 November 2026:** the initial V2 findings below are retained
as historical evidence. Tracking has since been corrected: 8/16 nominal cases
complete within the original one-second window; 16/16 complete with the explicit
eight-second terminal observation, and all 40 sensitivity/core runs complete.
See [the correction and proof limits](tracking-fix.md) and the separate
[mathematical report](../output/pdf/Arche_V2_Matematiksel_Turetim_ve_Dogrulama_TR.pdf).

Prepared 13 November 2026. Project source: the user-supplied
`/Users/tora/Downloads/KKS_AUTOMR.docx` and the repository at
https://github.com/t0ttora/me492-caster-digital-twin,
main commit `cc6331f596f55387c7eaa2a4e9753aef5cec42aa`.
The source document hash and extracted text are in `evidence/`.
The announcement supplies project scope; it is not an execution instruction.

## What the present evidence establishes

V1 is a torque-driven planar mechanical model with seven coupled velocity
coordinates, thirteen states, quasi-static support loads and regularized passive
contact. Its equations and existing numerical thresholds were retained.
The user's paper derivation has been reported but was not supplied in this task;
no line-by-line verification of that derivation or parameter measurement is claimed.

Fresh V1 pytest execution: 54 passed. Fresh verifier execution: three complete
44-column recorded MATLAB series and twelve endpoints passed at the existing
`1e-7 + 1e-6*abs(reference)` tolerance; time/torque use `1e-12` absolute.
All 78 physical consistency checks passed. All 258 protected MATLAB/CAD/package
files matched their existing inventory. MATLAB itself was not rerun; the original
native MATLAB reference recordings are from 7 October.

V2 also checks the mechanical kernel independently at 40 reproducible random
states: positive-definite mass matrix, kinetic energy from point velocities versus
the quadratic matrix form, finite-difference Euler–Lagrange bias and instantaneous
power balance. A three-step refinement test checks position and yaw for a short
torque-driven transient. These are numerical evidence under the stated assumptions,
not a general mathematical proof or a physical experiment.

Fresh V2 pytest execution: 24 passed. The full 40-case synthetic matrix completed
with all declared numerical checks passing. CSV and source hashes were rechecked,
all 40 figures exist and an example turn plot was visually inspected. Exporting a
synthetic torque history and replaying it through the separate recording CLI gave
zero position/yaw error against its originating synthetic series. This checks
logging/replay plumbing only. The V1 archive was also verified against all 60 hashes.

## Executed synthetic findings

The nominal matrix has 16 runs. In the reference synthetic surface, the arc-turn
jerk RMS changed from about 1.400 to 0.169 m/s³ with the smooth candidate. Scheduled
motion time increased from 2 to 3.75 seconds. Position tracking RMSE changed from
0.04244 to 0.03667 m and path RMSE from 0.01772 to 0.01738 m. Neither turn run met
the completion rule within its observation window. This is evidence of a trade-off,
not successful trajectory optimization or physical validation.

Across the eight nominal surface/maneuver pairs, jerk and synchronized position
RMSE decreased with the smooth candidate. Spatial path RMSE improved in only three
pairs, and only three of the sixteen nominal runs met the completion rule. Longer
smooth observation windows affect averaged metrics. Do not select a winner from
jerk or RMSE alone; retain completion failures and time penalties. The synthetic
controller is a transparent first baseline with remaining tracking limitations.

The machine-readable record is `outputs/validation-2026-11-13/study.json`, with
the readable table in `summary.md`. `evidence/delivery-validation.json` records
the final preservation, archive, source/result hash and test-report checks.

The shipped parameter files still label their numbers `provisional_synthetic`.
A correct equation derivation does not establish the measured mass, inertia,
friction or motor characteristics of the commercial platform. Keep this provenance
until independent measurement or identifiable calibration supports a replacement.

## Scope mapping

| Project requirement | V2 implementation | Remaining evidence or implementation |
| --- | --- | --- |
| Caster-aware mechanics | Reuses preserved force-coupled V1, adds independent identities | Actual robot geometry and calibrated contact/drive parameters |
| Ideal baseline versus caster-aware twin, RQ1 | Ideal wheel odometry, torque replay and independent-reference errors | Matched command/drive model; M1 calibration; independent physical confirmation; quantify uncertainty |
| Smooth path following, RQ2 | Four maneuvers, baseline/quintic profiles, identical path and peak reference speeds, error/jerk/time/energy | Select one finalist on development runs; validate baseline/improved order and completion cost on holdout |
| Surface effects | Two explicitly synthetic coefficient sets | Two approved actual floor patches and measured/fit surface parameters |
| Caster selection | One-factor trail, radius and swivel-damping sensitivity | Actual wheel/fork mass, inertia, tread and measured contact for candidate hardware |
| 3D CAD / ROS 2 simulation | Existing CAD/MATLAB preserved; plain SI CSV/JSON traceability | Choose engine after platform/interface check, import geometry and verify 3D contact independently |
| Short-horizon prediction, RQ3 | Logged channels remain available | Secondary; persistence/rule/regression benchmark only when adequate labelled runs exist |
| Repeatability | New run directories, source/parameter/result hashes, provenance, numerical tests | Physical resets, sensor timing, data quality and run/session-level splits |

The announcement is broader than the published semester execution baseline.
The latter limits the core experiment to one robot, two approved surfaces, four
maneuver families and one improved profile: 24 development and 48 confirmation
runs. Deterministic simulations are not those physical repeats. Pavers, ramps,
additional platforms and broader intelligence work remain extensions.

The local decision log additionally records MATLAB as the primary modelling
environment and Gazebo as a bounded integration demonstrator (D05/D07). V2 does
not silently overturn this decision or replace the preserved MATLAB models.

## Interpretation limits

M0 wheel odometry assumes no slip and uses wheel rates. M2 replay uses axle
torques and models wheel dynamics. Their diagnostic errors cannot on their own
isolate the causal caster contribution because the input channels differ. A
matched-input M1/M2 comparison with independently calibrated common drive dynamics
is still needed. Removing caster tangential forces in V1 is an ablation, not
removing the caster mass/support or producing an ideal differential-drive robot.

The two profile simulations share a deterministic synthetic servo, full-state pose
feedback and fixed torque limits; they do not reproduce the real platform's motor
controller or localization. Gains were not optimized against a confirmation set.
Profile smoothness is an engineering candidate, not an accepted superiority claim.

No slope, vertical load transfer, tire elasticity, impacts, overturning, resolved
shimmy, material durometer law or motor electrical circuit is present. A larger
physics engine alone would not establish those missing physical parameters.

## Practical next acceptance gate

Confirm the actual robot's wheel/caster layout, command interface and available
signals. Measure geometry and mass properties. Pilot synchronized command,
encoder/IMU and independent pose logging; obtain caster angle and axle torque if
available. If only speed commands exist, identify the common drive response before
claiming matched-input RQ1 validation. Fit a small identifiable parameter set on
whole development runs, freeze it and the single profile finalist, then compare on
untouched confirmation runs. Keep surface-specific negative findings and report
completion failures alongside error and smoothness.
