# Physical Experiment Protocol

One platform two surfaces and four maneuver families

Use two accessible, safe patches selected with Arche. Start with a smooth indoor surface and a second contrasting approved surface. Pavers, ramps and extra payloads are extensions after the core runs succeed. Record the exact patch rather than assigning universal friction values to a material name.

| Session request | Purpose and target |
| --- | --- |
| 20 or 21 Oct  2–3 hours | Pilot: check commands, timestamps, frames, pose reference, caster visibility and restart behavior. Pilot runs are separate from the final test. |
| 10 or 11 Nov  3–4 hours | Calibration: 2 surfaces × 4 maneuver families × 3 repeats = 24 nominal-profile runs. Use whole-run development splits. |
| 24 or 25 Nov  3–4 hours | Confirmation: 2 surfaces × 4 maneuver families × 2 profiles × 3 repeats = 48 runs. Balance baseline/improved order and seal final-test files. |
| 8 or 9 Dec  reserve 2–3 hours | Repeat only defective recordings or a specific unresolved comparison. If results guide tuning, collect a new untouched test set. |

These slots total 8–11 hours before the optional retry; all remain requests. Seventy-two main runs are a planning target. Three repeats per cell are an initial floor, not a statistical power guarantee: they are the minimum practical replication under the requested robot-access budget. Report uncertainty per run/cell rather than treating the repetitions as a powered population study. Typical runs take 30–90 seconds plus reset/checks. If resets exceed the slot budget, agree a reduction to two maneuver families before final-test collection.

## Measurements and controls

Record issued motion commands, wheel states/encoders, IMU, odometry, independent pose, source timestamps and logger timestamps where available. Prefer direct caster angle from an approved marker/video or sensor. Motor current is useful when available. Log geometry/configuration revision, payload, surface, battery state, operator and starting pose/caster angle for every run.

Validate reference scale and orientation against a known distance and a stationary test. Set sample rates from observed dynamics during the pilot. No shimmy claim follows from a signal whose sampling cannot resolve the oscillation. Preserve raw files and flags for gaps, resets and clock drift. Never use the odometry under evaluation as its own pose ground truth.

Arche sets safe speed, acceleration, area and stop rules. A designated operator supervises physical trials and can stop motion. I change only approved commands or profiles. Platform safety and low-level motor control remain under the existing system.
