# Arche JANUS ROS 2 and Gazebo

This workspace targets Ubuntu 24.04, ROS 2 Jazzy and Gazebo Harmonic. It includes the nominal robot, a world with one camera-facing QR marker, a bridge, a simulation-only command guard, camera observations and bag utilities. The Mac suite expands Xacro, parses XML/YAML and exercises the same Python helpers used by the ROS nodes. The Linux runner below records independent simulation state and rendered camera data for bounded native scenarios.

## Interfaces

| ROS topic | Message | Producer and intended use |
| --- | --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` | One-way bridge from `/world/janus/clock`. Header times and simulation replay use this domain. |
| `/janus/cmd_vel/request` | `geometry_msgs/msg/Twist` | One chosen upstream command source. Positive x is forward and positive z turns CCW. |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Command guard output and sole ROS-to-Gazebo drive input. This records the command sent to DiffDrive. |
| `/janus/abort` | `std_msgs/msg/Bool` | A true value latches zero output. A new process/run is required to clear it. |
| `/janus/command_guard/status` | `std_msgs/msg/String` JSON | Gate transitions, with the simulation timestamp and first latched fault. |
| `/odom` | `nav_msgs/msg/Odometry` | DiffDrive wheel odometry, with `odom` → `base_link`. |
| `/janus/ground_truth` | `nav_msgs/msg/Odometry` | Gazebo OdometryPublisher world pose and twist in `map`; it supplies no ROS TF. |
| `/joint_states` | `sensor_msgs/msg/JointState` | Two driven wheels, caster swivel and caster roll. Compare by joint name. |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Static identity `map` → `odom` for the nominal spawn, DiffDrive for `odom` → `base_link`, robot state publisher for child links. |
| `/janus/camera/image_raw`, `/janus/camera/camera_info` | `sensor_msgs/msg/Image`, `sensor_msgs/msg/CameraInfo` | The simulated front camera, both in `camera_optical_frame`. Live camera topics use sensor-data QoS. |
| `/janus/qr/observation` | `std_msgs/msg/String` JSON | One row per received frame, including rejections and all detected candidate quadrangles. |
| `/janus/replay/event` | `std_msgs/msg/String` JSON | Start and end or interruption marker with run ID, absolute simulation time and input SHA-256. |

The axle midpoint defines `base_link`, x is forward, y is left, z is up. World x/y is anchored at the nominal spawn. The V2 parameter source places its origin at the axle midpoint projected to the floor. Gazebo places `base_link` at axle height, with an initial 2 mm clearance for settling. Camera optical x points right, y points down and z points forward. The model inherits nominal wheel geometry, caster trail and lumped masses from V2, with splits and assumed three-dimensional inertias listed in [robot_config.json](src/arche_sim/config/robot_config.json). The approximate Gazebo contact and velocity actuation have their own dynamics. The visual target has no mechanical docking transform or contact/charger mechanism.

The QR JSON carries `capture_time_s`, `arrival_time_s`, `publish_time_s` in the Gazebo clock domain, image dimensions, target ID and column, selected `id`, four physical marker corners, projective center, area fraction, validity, reason and a `candidates` array. The corner order is physical top-left, top-right, bottom-right, bottom-left when upright. The rotated fixture checks that physical ordering is retained. Area fraction is a geometric quality measure. The normalized error is `(center_u_px - target_u_px) / width_px`; a positive value calls for negative yaw. The default target is `JANUS-DOCK-01` at column 320 of a 640 px image. The generated placard is 0.24 m wide including the quiet zone. The encoded 21-module symbol is approximately 0.174 m wide. Use the published intrinsics and actual marker transform for any later pose work. [OpenCV QRCodeDetector](https://docs.opencv.org/4.x/de/dc3/classcv_1_1QRCodeDetector.html)

The decoder rejects stale, duplicate, disordered and future frames. Its 0.20 s deadline is checked at arrival and again on publication. The image subscriber queues only the newest waiting frame, and the ROS executor can receive clock updates during decoding. A future frame can also represent bridge callback ordering. Candidate search works on decoded pixels. Severe degradation can obscure a candidate entirely. Consumers must also expire their latest accepted observation if callbacks cease. The Mac suite exercises an offline reference gate with ±2% acquisition, 0.4 s dwell, ±4% exit and immediate inhibition on invalid frames. The launch publishes observations for a future controller and contains no QR, BMS or mission controller. Use run-relative seconds for exported comparisons and subtract the same run-start clock from both capture and arrival. Restart with a fresh run ID after a clock reset.

The command guard publishes at 50 Hz using a steady timer. Its defaults are 0.25 s command timeout, 0.25 s odom timeout and 0.50 s wall-clock stall limit. A reset, regressive odometry, invalid numeric input or abort latches zero output. Encoder data is exported for diagnostics; wheel-joint-stream loss is an unmeasured acceptance condition. Keep exactly one publisher on the request topic during a replay, and keep the guard as sole publisher on `/cmd_vel`.

## Command guard and recovery

The guard sends zero while clock or odometry is unavailable, stalled, stale or too far ahead. A watchdog trip invalidates the stored request. Requests received while either input is unhealthy are discarded. After the inputs recover, the guard remains at zero (`command_timeout`) until it receives a new valid request. Clock regression, invalid or regressive odometry, invalid commands and operator abort latch until an explicitly initiated restart. An encoder-only loss detector is outside this guard.

For O08 after a loss of power or connectivity, stop or disable the old request publisher, restore and verify clock and odometry, obtain operator authorization, then start a fresh replay with a new run ID. The guard can check when it receives a Twist relative to its own input health. This unstamped message carries neither a sender timestamp nor proof of operator authorization. A publisher left running could send a new Twist automatically, and middleware queues can delay old messages. The offline O08 fixtures exercise direct guard calls for clock recovery, odometry recovery and a fresh guard instance after a latched abort. Native restart, queue behavior, operator authorization and stopping remain open.

The O08 simulation fixture enables `require_recovery_permission` on its isolated launch. A watchdog outage after motion clears the stored request and requires a separate `/janus/recovery/authorize` Boolean signal once clock and odometry are healthy. A fresh unauthorized `Twist` is discarded. The guard emits a `recovery_authorized` acknowledgment and requires a later request. The fixture records the world service acknowledgments, restored clock, unauthorized probe and authorization sequence independently. The default general launch keeps the permission fixture disabled.

On normal SIGINT or SIGTERM, the guard attempts one final zero on `/cmd_vel` before destroying the publisher when its ROS context is still valid. Process death, SIGKILL, a closed context, bridge or DDS loss, and concurrent downstream teardown can prevent the final zero from reaching the plant. No downstream command expiry is configured in this model. [Jazzy rclpy lifecycle and signal handling](https://github.com/ros2/rclpy/blob/jazzy/rclpy/rclpy/__init__.py)

## Mac offline reproduction

Use a writable work directory on the external SSD and a Python environment with Xacro, PyYAML, NumPy, OpenCV, qrcode and Pillow. The bench package makes synthetic PNGs and decodes those files through the application decoder. The station texture test exercises the build-generated QR asset as a PNG. Mac timings are warm per-image CPU decode wall times and exclude Gazebo rendering, ROS transit and camera capture. The source fixture choices and acceptance checks live in `fixtures.py`.

```sh
# Run from the repository root. Point the variables at the mounted SSD,
# the JANUS delivery outputs and the prepared Python environment.
ssd=/Volumes/your-external-ssd
work="$ssd/janus-ros2-offline"
shared=/absolute/path/to/janus-delivery/outputs
python=/absolute/path/to/venv/bin/python
test "$(stat -f %d "$ssd")" != "$(stat -f %d /Volumes)" || exit 1
mkdir -p "$work/tmp" "$work/cache"
export TMPDIR="$work/tmp/" XDG_CACHE_HOME="$work/cache"
export PYTHONPYCACHEPREFIX="$work/cache/pycache" PYTHONDONTWRITEBYTECODE=1
cd janus/ros2 || exit 1
PYTHONPATH="$PWD/src/arche_experiments" "$python" -B -m unittest discover -s tests -v
"$python" -B bench.py --suite \
  --fixtures "$shared/scenarios/commands" \
  --python-dir "$shared/scenarios/commands/python" \
  --plan "$shared/report-source/approved-plan.md" --contract "$shared/scenarios/contract.md" \
  --out "$work/reproduction" --repeats 30
```

The benchmark can run against the M01 fixtures immediately. Omit `--python-dir` for that pass. On import, it checks each Python producer CSV against its manifest, retains its declared initial pose and records each stream by role. It integrates velocity at 20 ms plus exact command edges using zero-order hold and analytic no-slip kinematics. Requested velocity and limited wheel-PI reference are run independently. Torque streams are checked for schema, time, finite values and checksum. Effort comparison requires a native effort-actuated model and applied-effort logging.

Each benchmark directory has a 40-case `case_results.csv`, per-execution `runs.csv` and `manifest.json`, plus fixture or command data, scope-specific results and metrics. Paths inside each run manifest are relative to that run directory. `partial` means at least one relevant subsidiary check was exercised. Inspect each `check_status` and the plan acceptance condition independently. Empty physical columns and JSON nulls indicate unavailable measurements.

## Ubuntu 24.04 native setup

Configure the official Jazzy ROS apt repository and install Jazzy before using the sequence below. `ros-jazzy-ros-gz` selects the supported Harmonic vendor packages. Use a single ROS and Gazebo installation on the host. [Jazzy Ubuntu binaries](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) · [Harmonic ROS installation](https://gazebosim.org/docs/harmonic/ros_installation/) · [Jazzy ros_gz bridge](https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_bridge)

```sh
sudo apt update
sudo apt install build-essential ros-jazzy-ros-base ros-jazzy-ros-gz ros-jazzy-xacro ros-dev-tools \
  ros-jazzy-rosbag2-storage-mcap python3-qrcode python3-pil \
  python3-opencv python3-numpy python3-yaml
source /opt/ros/jazzy/setup.bash

# Source points to this README's directory. Work is a fresh writable directory.
source_tree=/absolute/path/to/checkout/janus/ros2
work=/absolute/path/to/janus-linux-run
mkdir -p "$work"/{cache,tmp,ros-log,runs}
export TMPDIR="$work/tmp" XDG_CACHE_HOME="$work/cache" ROS_LOG_DIR="$work/ros-log"
cd "$work"
# Initialize rosdep once on a new host with sudo rosdep init, then run rosdep update.
rosdep install --from-paths "$source_tree/src" --ignore-src --rosdistro jazzy -y
colcon --log-base "$work/colcon-log" build --base-paths "$source_tree/src" \
  --build-base "$work/build" --install-base "$work/install"
source "$work/install/setup.bash"
ros2 launch arche_bringup janus.launch.py
```

The launch starts the world with the predeclared robot and marker models. It starts Ogre2 in server mode with headless rendering. On a working graphical session, pass `gz_flags:="-r -v 3"` for the GUI. If Ogre2/EGL fails, retain the error output and leave rendered-camera acceptance pending. Before driving the world, check these interfaces from another sourced terminal. [Gazebo Harmonic sensors](https://gazebosim.org/docs/harmonic/sensors/) · [Gazebo Sim 8 DiffDrive](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/diff_drive/DiffDrive.cc)

```sh
ros2 topic info /clock --verbose
ros2 topic info /cmd_vel --verbose
ros2 topic echo /odom --once
ros2 topic echo /janus/ground_truth --once
ros2 topic echo /joint_states --once
ros2 topic echo /janus/camera/camera_info --once --qos-reliability best_effort
ros2 topic echo /janus/qr/observation --once
ros2 run tf2_ros tf2_echo map base_link --ros-args -p use_sim_time:=true
```

Confirm one ROS publisher on `/clock` and on `/cmd_vel`, monotonic time, one `odom` → `base_link` TF authority, all four joint names, matching camera frames and the decoded station ID. Validate M01 idle first. Then use distinct run IDs for forward, reverse and positive yaw. Check wheel signs and motion against the independent world topic. In the native camera smoke, inspect the recorded image and verify the orientation of the rendered texture. Confirm the expected clock and frame stream before starting a bag.

### Isolated case runner

In a sourced Linux overlay, run one case per new output directory. The shell runner assigns a DDS domain and a Gazebo partition per invocation, and shuts down its dedicated process group. Keep any unrelated local ROS applications on a different domain. Set `JANUS_ROS_DOMAIN_ID` to a known unused ID when rerunning in a shared environment. The default launch starts the QR observer. The mechanical and command-fault fixtures disable that observer without changing the production default.

```sh
bash experiments/run-native-case.sh M02 /absolute/path/to/new-run-M02
python3 experiments/validate-native.py /absolute/path/to/new-run-M02
```

The runner records measured Gazebo world pose, simulated odometry, measured wheel/caster joints, command-guard output and raw experiment events. QR case outputs also retain rendered camera frames and decoded observations. Q13 interrupts the camera bridge and reinjects a captured frame with its original timestamp. O08 pauses and resumes the live Gazebo clock and separately issues an explicit new request. These faults exercise the isolated simulator and ROS topics. The simulator supplies wheel-velocity actuation and these runs make no claim about applied axle effort or physical robot stopping distance.

The isolated QR fixtures use the `zbar` backend after `sudo apt install python3-pyzbar`. It reads actual rendered QR content before the common identity, freshness and centering checks. The runner selects a single-marker fast path only for the world with the named robot and sole nominal marker. Unknown or multi-marker scenes use the additional OpenCV residual check. Select the default OpenCV pipeline with `JANUS_DECODER_BACKEND=opencv`. The observer logs the backend and fixture setting in the launch evidence. Use `JANUS_RECORD_BAG=1` to keep a compressed MCAP and exported topic trace as part of the run.

Q13's `JANUS_Q13_MODE=duplicate` run interrupts its camera bridge and republishes a retained, expired render. `JANUS_Q13_MODE=regression` remaps the native camera through its test relay, delays no simulation clock and sends a distinct older, still-fresh captured image after the observer has accepted a newer frame. The `resume` variant additionally waits for measured motion after new valid observations. Each run writes its detector or guard reasons alongside measured state.

For a Q13 `duplicate` run, `python3 experiments/validate-native.py /absolute/path/to/new-run-Q13 --camera-flow-stall` records the freshness watchdog, zero command and independent stop that occur before the retained frame is injected. It writes the supplemental `camera-flow-stall-acceptance.json`.

O05 normally latches regressive odometry. `JANUS_O05_RECOVERY=1` opts this isolated fixture into recovery. It injects the previous simulated odometry packet with its timestamp rewound by 250 ms, checks the stopped hold while fresh odometry advances, and requires an operator authorization event followed by a separate new command before motion restarts. Use `python3 experiments/validate-native.py` with the O05 output to verify those steps.

`JANUS_CAMERA_VARIANT=near_start` uses an explicitly recorded initial pose and a wall-rate target of 0.25 for smaller guest scheduling load. It preserves the 1 ms physics step, 20 Hz simulation-camera setting and acceptance gates. The optional `JANUS_CONTROL_DEPTH_M` sets a controller target inside the fixed optical acceptance band. The acceptance script recomputes optical depth from the independent Gazebo pose. Set `JANUS_CONTROL_DEPTH_M=.261` for the measured Q03 margin variant.

```sh
JANUS_CAMERA_VARIANT=near_start JANUS_Q13_MODE=regression \
  bash experiments/run-native-case.sh Q13 /absolute/path/to/new-run-Q13
python3 experiments/validate-native.py /absolute/path/to/new-run-Q13
python3 experiments/plot-native-case.py /absolute/path/to/new-run-Q13
```

## Native bag, replay and benchmark

All three terminals need the Jazzy and workspace environments. Set `case_id`, `replay_file`, `run_id` and `run_dir` consistently. For a first M01 run use the shared `M01-zero-velocity.csv`. For M02 use the requested or limited velocity input from its upstream manifest and record which input was used. Include the exact fixture schedule, initial state, model/command hashes, versions, start/end clock and termination reason with the bag. Choose initial poses and caster state appropriate for the scenario before assigning a case-level result.

```sh
case_id=M01
run_id=M01-gazebo-native-smoke-001
replay_file=/absolute/path/to/M01-zero-velocity.csv
run_dir="$work/runs/$run_id"
mkdir -p "$run_dir"
sim_share="$(ros2 pkg prefix --share arche_sim)"
cp "$replay_file" "$run_dir/commands.csv"
cp "$sim_share/config/robot_config.json" "$run_dir/robot_config.json"
sha256sum "$run_dir/commands.csv" "$sim_share/worlds/janus.sdf" \
  "$sim_share/models/arche_janus/model.sdf" "$sim_share/models/janus_dock/meshes/marker.png" \
  "$sim_share/config/bridge.yaml" "$sim_share/config/camera_bridge.yaml" > "$run_dir/input-sha256.txt"
( uname -a; gz sim --versions; printenv ROS_DISTRO; dpkg-query -W 'ros-jazzy-ros-gz*' ) > "$run_dir/versions.txt"

# Terminal 2. Let the recorder discover topics and receive /clock before replay.
ros2 bag record --use-sim-time -s mcap -o "$run_dir/bag" \
  --qos-profile-overrides-path "$sim_share/config/bag_qos.yaml" \
  /clock /tf /tf_static /odom /joint_states /janus/ground_truth \
  /janus/cmd_vel/request /cmd_vel /janus/command_guard/status /janus/abort \
  /janus/camera/image_raw /janus/camera/camera_info /janus/qr/observation /janus/replay/event

# Terminal 3.
ros2 run arche_experiments replay_velocity "$run_dir/commands.csv" --run-id "$run_id"
```

Record briefly after the replay ends, then stop the recorder cleanly. Replay uses simulated time and holds each CSV row until the next timestamp, with a sample-and-publish timer of nominally 10 ms wall time. It requests zero at the terminal CSV timestamp. Compare the actual ROS command topics because scheduler and transport timing can shift a step. For a command-timeout trial, stop the request publisher during nonzero motion with the guard and recorder still running. For an abort trial publish `std_msgs/msg/Bool` true to `/janus/abort` and check the latched output and measured stop. [Jazzy rosbag2](https://github.com/ros2/rosbag2/tree/jazzy)

```sh
ros2 bag info "$run_dir/bag" > "$run_dir/bag-info.txt"
ros2 run arche_experiments export_bag "$run_dir/bag" "$run_dir/trace.jsonl"
python3 "$source_tree/bench.py" --case "$case_id" --commands "$run_dir/commands.csv" \
  --trace "$run_dir/trace.jsonl" --native-run-id "$run_id" --out "$run_dir/benchmark"
```

Trace analysis checks the run ID and input SHA-256 and requires at least two advancing clock timestamps inside the marked replay interval. The 20 ms grid lies within the overlap of those clock samples, odometry, Gazebo world pose and joint samples. Metrics include the in-run clock span and largest observed clock gap, state gaps and odom disagreement. The analyzer unwraps continuous angles and uses zero-order hold between in-run command samples at recorded simulation receive times. The bag-trace check fails when either the request or guard-output topic covers zero grid samples. An available state result is still exported for diagnosis. A passing check establishes sampled bag topic overlap. Bridge delivery, applied effort, timing through any gaps and case-specific acceptance require their own measurements. [Gazebo Sim 8 OdometryPublisher](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/odometry_publisher/OdometryPublisher.cc)

For an offline ROS camera transport replay, stop Gazebo and the live bridge. Run a fresh observer with `use_sim_time:=true`. Play only the recorded clock and camera topics, with no generated `--clock` and no recorded observation or command topics. This replay tests message and image plumbing. Closed-loop QR evidence requires commands to move the plant and current camera observations to respond to that motion. [Jazzy rosbag2 playback](https://github.com/ros2/rosbag2/tree/jazzy)

```sh
ros2 run arche_experiments qr_observer --ros-args -p use_sim_time:=true
# From another sourced terminal, after starting the observer.
ros2 bag play "$run_dir/bag" --topics /clock /janus/camera/image_raw /janus/camera/camera_info
```
