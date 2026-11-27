#!/usr/bin/env bash
set -eo pipefail
cd "${1:?guest staging directory}"
export ROS_DOMAIN_ID=83 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=arche_janus_video_83 GZ_IP=127.0.0.1
export LIBGL_ALWAYS_SOFTWARE=1 EGL_PLATFORM=surfaceless LP_NUM_THREADS=2
source /opt/ros/jazzy/setup.bash
source "$HOME/janus-build/install/setup.bash"
export PYTHONPATH="$PWD/source:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
share="$HOME/janus-build/install/arche_sim/share/arche_sim"
python3 scripts/prepare_scene.py "$share" scene
export GZ_SIM_RESOURCE_PATH="$PWD/scene/models"
export ROS_LOG_DIR="$PWD/native-logs/ros"
mkdir -p "$PWD/native-logs" "$ROS_LOG_DIR"
gz sim -r -s --headless-rendering -v 4 "$PWD/scene/two_station.sdf" >native-logs/gz.log 2>&1 &
gz_pid=$!
"$(ros2 pkg prefix ros_gz_bridge)/lib/ros_gz_bridge/parameter_bridge" --ros-args -p config_file:="$share/config/bridge.yaml" >native-logs/bridge.log 2>&1 &
bridge_pid=$!
"$(ros2 pkg prefix ros_gz_bridge)/lib/ros_gz_bridge/parameter_bridge" --ros-args -p config_file:="$share/config/camera_bridge.yaml" -p override_frame_id:=camera_optical_frame >native-logs/camera-bridge.log 2>&1 &
camera_pid=$!
cleanup() {
  kill -TERM "$camera_pid" "$bridge_pid" "$gz_pid" 2>/dev/null || true
  wait "$camera_pid" "$bridge_pid" "$gz_pid" 2>/dev/null || true
}
trap cleanup EXIT
gz sim --versions > native-logs/gazebo-version.txt
gz topic -l > native-logs/gazebo-topics-initial.txt
python3 scripts/capture_native.py capture/mixed_ids
gz topic -l > native-logs/gazebo-topics.txt
pose() {
  gz service -s /world/janus/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean     --timeout 3500 --req "name: \"$1\", position: {x: $2, y: $3, z: 0.31}, orientation: {w: 1}"
}
pose station_other 1.50 -5.5 | tee native-logs/pose-remove-other.log | grep -F 'data: true'
pose station_duplicate 1.50 -0.44 | tee native-logs/pose-place-duplicate.log | grep -F 'data: true'
sleep 1
python3 scripts/capture_native.py capture/duplicate_ids
pose station_duplicate 1.50 -5.5 | tee native-logs/pose-remove-duplicate.log | grep -F 'data: true'
pose station_primary 0.61 -0.23 | tee native-logs/pose-edge.log | grep -F 'data: true'
sleep 1
python3 scripts/capture_native.py capture/near_edge
pose station_primary 1.30 0 | tee native-logs/pose-clean.log | grep -F 'data: true'
sleep 1
python3 scripts/capture_native.py capture/clean_native
