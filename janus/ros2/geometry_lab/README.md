# JANUS native geometry laboratory

This runner isolates M07, M11, M12, O02, O03, and Q14 on the integration Gazebo chassis. It uses the standard `ros_gz_bridge`, integration `command_guard`, and Gazebo DiffDrive. The cases publish laboratory Twist requests and observe body motion from the independent Gazebo world odometry. Fixtures, physics, observer provenance, and load measurement remain within simulation.

## Environment

The validated environment is Ubuntu 24.04 arm64, ROS 2 Jazzy, Gazebo Harmonic 8.15.0, DART, the Jazzy Gazebo vendor Python bindings, and a writable SSD-backed Lima virtiofs mount. Source `/opt/ros/jazzy/setup.bash` and the existing integration overlay. The C++ systems use the Gazebo development headers from the Jazzy vendor packages. Python requires NumPy for reduction and Matplotlib for plots. [Gazebo Harmonic ROS installation](https://gazebosim.org/docs/harmonic/ros_installation/) · [Gazebo 8 contact system](https://gazebosim.org/api/sim/8/triggeredpublisher.html)

The lane runner expects `/mnt/janus-evidence/gazebo-geometry` with `arche_sim` copied from `src/arche_sim`, a `runs` directory, and the source scripts staged together. Copy the integration build-generated `models/janus_dock/meshes/marker.png` against its matching DAE for Q14. Stage the unmodified integration scripts and Python package under `integration-629bf31`. This pins the command guard and stock observer to the case baseline independently of other builds in the shared guest. The ordinary integration installation remains read-only. Run one case at a time on ROS domain 84. The runner applies a domain lock and a unique Gazebo transport partition per run.

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/janus-build/install/setup.bash"
lane=/mnt/janus-evidence/gazebo-geometry
cmake -S "$lane/src-v8" -B "$lane/instrumentation-v3/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$lane/instrumentation-v3/build" --parallel 1
bash "$lane/src-v8/run_guest.sh" M07 replanned M07-gazebo-example
python3 -B "$lane/src-v8/metrics.py" "$lane/runs/M07-gazebo-example"
python3 -B "$lane/src-v8/plot_evidence.py" "$lane/runs/M07-gazebo-example" "$lane/runs/M07-gazebo-example/evidence.png"
```

Rerun the reducer after plotting to include the PNG in the SHA-256 manifest. Keep a distinct run ID for each run and retain every interrupted or failed candidate.

## Fixtures and measured quantities

| Case | Fixture and measurement |
| --- | --- |
| M07 | Collidable walls x=[−0.34, 0.74] m and y=[−0.36, 0.76] m. Original 100 mm fillet, translated original, and 60 mm replan use explicitly declared spawn and segment schedules. Native contact, cross-track error, terminal dwell, and two projected clearance calculations are retained. |
| M11 | Collision seam x=0.30 m. Boundary friction 0.60/0.12 and homogeneous seam controls. Uses native wheel/floor contact times, wheel and body velocities, stop time, and stop distance. |
| M12 | Finite 3° ramp and 240 mm long finite raised threshold. Uses chassis-to-terrain vertical clearance, full crossing position, raw contact wrench, and direct contact presence at every 1 ms step. |
| O02 | 0.35 m entry followed by a 180° turn. The original route leaves forward after turning. A separate literal reverse-gear variant includes a pivot back to the inward heading. Both measure swept clearance, contact, terminal dwell, and caster recovery. |
| O03 | The contact-capable stand-in moves laterally across the corridor on Gazebo's clock. A separate ROS process publishes the laboratory hazard and optional consent. Clear is derived from recorded native obstacle pose. |
| Q14 | The moving collision obstacle blocks the renderable dock corridor. The approach uses the real camera/QR observation, with existing ±2% and ±4% gates, 0.40 s acquisition and 0.20 s freshness. Lab hazard and consent are separately injected. Recovery observer variants use a frozen local rectification candidate in `qr-recovery`. The `qr-runtime` variant uses a separately pinned runtime-owner observer under `runtime-snapshot-0358`, with the base command guard. |

The M07 and O02 upstream rectangle uses 0.25 m × 0.205 m half extents. The SDF projection uses the chassis box, drive-wheel cylinders and caster wheel cylinder in measured pose, including measured swivel. The fixture separation for O03 and Q14 is a conservative lower bound from synchronized AABB projections. Native contact is a separate measured stream.

The stock Gazebo contact publisher omits empty steps. The direct system records both positive and zero-contact steps without relying on receipt over transport. Its force columns are the solver's collision-point wrenches summed on the robot collision. It does not infer applied axle torque. Transient force peaks depend on the solver, geometry, and time step. [Gazebo 8 contact publisher source](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/contact/Contact.cc) · [Gazebo 8 physics contact conversion](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/physics/Physics.cc)

Q14 uses target real-time factors 0.055 or 0.12, declared per observer variant, to accommodate decoder latency in the shared arm64 guest. The simulation clock still governs the same dynamics step and freshness and dwell thresholds. Obstacle pose, hazard input, operator stand-in, guard output, robot braking and motion resume are distinct records. The lab does not supply a company obstacle-recognition policy, production operator authentication, or hardware qualification. Check scenario-owner clearance and load limits independently of these measurements.

## Raw formats

- `trace.jsonl` records the ROS source and capture/receipt timestamp where provided. The clock is absolute Gazebo simulation seconds. Subtract the origin in `termination.json` for comparisons.
- `contact-*.pb.gz` consists of repeated little-endian uint32 message length followed by the corresponding serialized `gz.msgs.Contacts`.
- `native_contact_step.csv` records physics steps for the chassis and each wheel. `fixture_native_pose.csv` records the obstacle's actual and requested lateral poses separately at 50 Hz.
- `bridge.yaml`, `scene.sdf`, `fixture.json`, `versions.txt`, and `input-sha256.txt` pin the setup. `ros_publisher_graph.json` records discovered ROS publishers for later instrumented runs.
- `derived.csv`, `support.csv`, and `metrics.json` reduce the raw streams. `manifest.json` identifies each evidence file by digest.
