# Recording contract

V2.2 uses the V2 `parameters/` working set (drive spin inertia 0.001125 and
caster roll inertia 0.000320 kg m²). To reproduce a V1/V2.1 recording or export,
specify its original effective parameters explicitly in `parameter_overrides`;
do not replay an old synthetic fixture against changed defaults and expect parity.

The evaluator is an offline diagnostic. It never connects to or drives a robot.
Keep original raw recordings untouched and resample a derived copy only after
checking clock synchronization, dropped samples, units and frame transforms.

## CSV

Exactly one row per command interval, default 0.01 seconds, starting at zero.
The default solver step is 0.0005 seconds. All values must be finite. Gaps and
nonuniform grids are rejected rather than silently interpolated. Columns:

```csv
time_s,left_drive_torque_Nm,right_drive_torque_Nm,left_wheel_rate_radps,right_wheel_rate_radps,reference_x_m,reference_y_m,reference_yaw_rad
```

Torque in a row applies from that timestamp to the next row (ZOH). The last
row's torque is logged but has no future integration interval. Rates are wheel
angular rates in rad/s, positive forward. Yaw is counterclockwise positive.
Reference pose is an independent world-frame measurement of the drive axle
midpoint, never wheel odometry. Transform a camera/body-marker pose using its
known offset before calling the evaluator. Odometry uses interval-average wheel
rates and exact constant-curvature displacement; this is a sampled approximation.

## Metadata

Example structure below is **synthetic**, not a real run. Fill every field from
the actual source and use one file per run. Do not mark a synthetic log measured.

```json
{
  "run_id": "synthetic-example-001",
  "session_id": "synthetic-session",
  "split": "development",
  "data_kind": "synthetic",
  "surface_id": "reference_synthetic",
  "robot_revision": "v1-synthetic",
  "pose_source": "synthetic_fixture",
  "pose_frame": "drive_axle_midpoint_world_SI",
  "torque_units": "axle_Nm",
  "torque_source": "synthetic_program",
  "parameter_provenance": "V1 provisional synthetic defaults",
  "initial_state": [-0.08, 0, 0, 0.7853981633974483, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  "parameter_overrides": {"geometry": {}, "dynamics": {}}
}
```

`split` is `development` or `confirmation`. `data_kind` is `synthetic` or
`measured`. Initial state ordering is the V1 State contract:

1. Body COM world x m
2. Body COM world y m
3. Body yaw rad
4. Caster angle relative to body rad
5. Body COM world x velocity m/s
6. Body COM world y velocity m/s
7. Body yaw rate rad/s
8. Caster relative rate rad/s
9. Left wheel rate rad/s
10. Right wheel rate rad/s
11. Caster rolling rate rad/s
12. Integrated motor work J, normally zero at the start of a run
13. Integrated dissipated energy J, normally zero at the start of a run

Unknown initial caster orientation/rates are a source of uncertainty, not zero
by default. Record the measurement or assumption. The evaluator checks that the
initial axle pose and driven wheel rates agree with the first log row.

Overrides must use existing Geometry/Dynamics field names and pass V1 validation.
Use `parameter_provenance` to identify the measurement/fit and whole development
runs used. Merely overriding a number does not make it calibrated. Freeze fitted
parameters before confirmation; the evaluator performs no fit or model selection.

The result records source hashes, effective parameters and metadata. A successful
evaluation remains a per-run numerical comparison, with `real_robot_validated`
false; a validation claim additionally needs protocol quality checks, matched
model inputs, independent confirmation runs, uncertainty and acceptance criteria.
The synthetic self-replay test checks plumbing, not physical accuracy.

When only wheel rates and independent pose exist, M0 odometry can be evaluated
using `twin.wheel_odometry` and `twin.pose_errors`. M2 torque replay additionally
requires reliable axle torques and the initial dynamic state. Do not derive
ground truth from the encoder rates being evaluated.
