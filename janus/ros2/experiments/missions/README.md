# JANUS Gazebo mission evidence

The harness runs Q15, Q16, Q18, O01, O06 and O07 on Ubuntu 24.04 arm64 with ROS 2 Jazzy and Gazebo Harmonic. It uses the baseline JANUS DiffDrive plant, ROS command guard and camera transport. The station, selectable cut authorization and battery readings are laboratory fixtures. The Linux teardown shuts down only the process group it started.

## Reproduce on the existing SSD-hosted Lima guest

Build from this checkout's `janus/ros2/src` on the Jazzy guest. Set `ROS_SOURCE` and `MISSION_SRC` to their guest-visible paths. On the existing Lima setup both trees can be copied into a new subdirectory of the writable evidence mount before executing these commands.

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p "$HOME/janus-mission-build"
cd "$HOME/janus-mission-build"
colcon build --executor sequential --base-paths "$ROS_SOURCE" --cmake-args -DBUILD_TESTING=OFF
bash "$MISSION_SRC/prepare_codecs.sh"
mkdir -p "$JANUS_EVIDENCE"
for case in Q15 Q16 Q18 O01 O06 O07; do
  bash "$MISSION_SRC/run_guest.sh" "$case" "$JANUS_EVIDENCE/$case" "$PWD/install/setup.bash" 190
  python3 "$MISSION_SRC/verify_mission.py" "$case" "$JANUS_EVIDENCE/$case"
done
```

The codec script installs no system packages. It checks both Ubuntu Noble Universe arm64 package digests before extracting under the guest lane's directory. The harness uses domain 82, localhost DDS, and a unique Gazebo partition and world per case. Reserve domain 82 for the serial invocation. Each output directory must be new. The runner stores explicit command lines, tool and solver versions, hashes, actual spawn world, input manifest, execution results and raw JSONL. `fixture_loop_finished` records termination of the run loop. The verifier reports case acceptance.

The verifier is free of ROS and Gazebo dependencies.

```bash
python3 -m unittest discover -s "$MISSION_SRC" -v
python3 "$MISSION_SRC/verify_mission.py" Q15 "$JANUS_EVIDENCE/Q15"
python3 "$MISSION_SRC/plot_missions.py" "$JANUS_EVIDENCE/charts" \
  Q15="$JANUS_EVIDENCE/Q15" Q16="$JANUS_EVIDENCE/Q16" Q18="$JANUS_EVIDENCE/Q18" \
  O01="$JANUS_EVIDENCE/O01" O06="$JANUS_EVIDENCE/O06" O07="$JANUS_EVIDENCE/O07"
```

## Fixture and measurements

- Q15, Q16 and Q18 use a near-field staging spawn at x = 0.76 m. The station tag is at x = 1.50 m and the camera is 0.25 m ahead of the axle. The declared 0.265 m optical standoff follows the native lane's camera approach fixture. The independent terminal reference is x = 0.985 m. The robot still advances under decoded live feedback with the approved centering bands, 0.4 s dwell, and 0.20 s frame freshness. This fixture covers station behavior after staging.
- A separate virtual station ROS node supplies synthetic contact/BMS/SoC messages with capture timestamps and sequence numbers. Contact is conditioned on measured Gazebo alignment and is published after the zero-motion event. Q15 withholds charge. Q16, Q18 and O07 include a pre-contact packet, a packet whose capture follows wait start but whose transmission is already stale, and a fresh acknowledgement. The verifier recomputes geometry, motion, the full timeout or packet freshness and event order from raw samples.
- Q18 uses explicit lab authorization, a cut command, then a fresh cut acknowledgement. Negative Twist is enabled on a later tick. The small initial steering seed breaks symmetric caster equilibrium. The Gazebo plant supplies base pose and caster joint angle/rate. The verifier computes conservative planar distance to collidable guide boxes using chassis, drive-wheel and caster footprints. The caster gate requires error within 0.15 rad of the reverse direction and swivel rate within 0.10 rad/s for 0.25 s while the body is moving backward. The station contact signal and company approval remain separate scopes.
- O01 and O06 use wheel odometry for waypoint control and independent Gazebo world pose for arrival gates. O01 issues three task IDs and resets goal and dwell state for each. O06 adds a rigid 4 kg payload at `[-.03, 0, .145]` m relative to the base. The generated world declares the payload, its inertias and its fixed joint, and publishes native floor contacts for the drive wheels and caster.
- O07 starts a distinct transit mission while SoC is 55%. The declared 15% threshold is crossed by a synthetic 12% reading while transit is active. The controller stops the plant, dispatches return-to-dock and runs the live QR and charge flow.

ROS /clock is the event timebase. Stamped world pose, wheel odometry and joints retain producer timestamps. Request and guard Twist rows use receiver-side ROS time. The traces retain absolute simulation stamps and charts show case-relative or phase-relative time. The baseline actuator exposes Twist and joint velocities. Applied axle torque and physical station/BMS behavior remain unmeasured.
