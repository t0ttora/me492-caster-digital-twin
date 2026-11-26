# JANUS offline navigation dataset contract

## One record per sensor frame

An episode has `episode_id`, `split`, `case_id`, `variant_id`, `seed`,
`robot_config_sha256`, `scenario_sha256`, `route_id`, `camera_config_id`,
`station_layout_id`, initial `x_m,y_m,yaw_rad,caster_relative_rad`,
and an explicit `source` (Gazebo, Python, Simscape, or authorized pilot).
Record the generating code/model version and whether the trajectory is shared
across engines. Start a new episode when the clock resets. Group the whole
episode, layout and overlapping footage within one data split.

| Stream | Join key | Minimal payload |
| --- | --- | --- |
| Camera frame | episode_id, frame_id | image path/hash, capture_time_s, arrival_time_s, width_px, height_px, intrinsics, distortion and body-to-camera transform references |
| QR observations | episode_id, frame_id, detection_index | decoded ID, four ordered marker corners, center, normalized horizontal error, visibility, measurement quality and reject reason |
| State | episode_id, time_s, state_source | x_m, y_m, yaw_rad, forward speed and yaw rate, pose frame, covariance if supplied, estimator/ground-truth kind |
| Teacher command | episode_id, time_s | v_mps, yaw_rate_radps, validity interval, proposed and limited values, teacher ID, action provenance |
| Native replay labels | scenario hash, run_id, time_s | caster angle/rate, wheel speeds/torques, penetration/normal forces, convergence flag, phase, hard-case labels |
| Outcomes | episode_id, event_time_s | phase, aligned, contact, charge_confirmed or abort/timeout, authoritative cause and evidence source |

Use SI, axle midpoint x forward/y left/z up and CCW yaw. Camera optical axes
are right/down/forward. Action proposals are waypoints in the body frame at
capture time, expressed in metres with an explicit horizon/spacing and heading
convention. A deterministic downstream translator bounds speed and yaw, while
the QR controller retains final approach, identity, freshness, stop and charge
interlocks. A policy candidate stays offline during this experiment.

## Join and eligibility

1. Generate a Gazebo render sequence with fixed fixture, camera and route.
   Record image, camera calibration, simulated state, commands, pose source and
   simulation timestamps. The Linux rendering run is needed to produce actual
   Gazebo images.
2. Obtain teacher proposals using the logged state/QR and reference path
   available at capture. Recompute the Python teacher on that causal state if
   replay is on a different trajectory. A Python closed-loop CSV can provide
   a sample-level target only after matching/configuring the same initial
   conditions and certifying pose alignment. Copying commands merely by
   identical relative timestamps would mislabel diverged trajectories.
3. Join native hard-case tags such as high caster lag, reversal, contact onset
   and swivel solver sensitivity by scenario hash plus run and time window.
   Native force/caster labels are scenario-level context until time, frames,
   command type and achieved trajectory have been aligned and checked. Compare
   effort replay on applied torque series and trajectory replay on explicitly
   labeled speed commands. Set unaudited per-frame labels to missing.
4. Freeze station geometry, transforms, timestamp residual limits and train/
   development/confirmation groups before a held-out run. Do not tune a
   candidate with its held-out scenarios. Report frame loss and timestamp gaps,
   reject stale or ambiguous detections and keep invalid frames as negatives
   with reasons. Preserve raw inputs, derived data and package checksums.
5. Compare candidate and classical stack on the same held-out frames and
   episodes. Report per-episode staging success, false station approach, QR
   lost-command delay, clearance/intervention, waypoint displacement/heading
   error and p50/p95 inference latency with hardware and pipeline scope. Use
   paired episode summaries and confidence intervals; never treat correlated
   video frames as independent trials. Physical calibration and confirmation
   run on separate operator-approved pilot sessions.

Example join direction, with placeholders: `Gazebo episode E03 / frame F012`
links to the same episode's most recent causal teacher command. The matched
scenario's Simscape reversal tag is attached as scenario context; a timed
contact-force label is attached only if replay alignment passes. This example
specifies the data contract and contains no actual camera images.
