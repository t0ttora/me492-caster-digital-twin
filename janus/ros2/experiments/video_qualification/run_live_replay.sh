#!/usr/bin/env bash
set -eo pipefail
cd "${1:?guest staging directory}"
out="${2:?output directory name}"
shift 2
export ROS_DOMAIN_ID=83 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=arche_janus_video_83 GZ_IP=127.0.0.1
source /opt/ros/jazzy/setup.bash
source "$HOME/janus-build/install/setup.bash"
export PYTHONPATH="$PWD/source:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR="$PWD/$out/ros"
export GZ_SIM_RESOURCE_PATH="$PWD/scene/models"
export OPENCV_FOR_THREADS_NUM=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
mkdir -p "$out" "$ROS_LOG_DIR"
share="$HOME/janus-build/install/arche_sim/share/arche_sim"
sha256sum source-nodes/{qr_observer,command_guard} \
  source/arche_experiments/{vision,commands,ros_fields}.py scripts/{live_replay.py,run_live_replay.sh} \
  "$share/config/bridge.yaml" "$GZ_SIM_RESOURCE_PATH/arche_janus/model.sdf" fixtures/manifest.json \
  > "$out/runtime-inputs.sha256"
# The solver step stays fixed. A throughput cap only changes elapsed wall time.
python3 - "$out/physics.sdf" "${JANUS_VIDEO_REALTIME_FACTOR:-1.0}" <<'PY_WORLD'
import sys
import xml.etree.ElementTree as ET
tree = ET.parse("scene/physics_only.sdf")
tree.find("world/physics/real_time_factor").text = sys.argv[2]
tree.write(sys.argv[1], encoding="utf-8", xml_declaration=True)
PY_WORLD
# The source robot, dynamics, bridge and nodes are the pinned runtime inputs.
gz sim -r -s -v 4 "$PWD/$out/physics.sdf" >"$out/gz.log" 2>&1 &
gz_pid=$!
"$(ros2 pkg prefix ros_gz_bridge)/lib/ros_gz_bridge/parameter_bridge" --ros-args -p config_file:="$share/config/bridge.yaml" >"$out/bridge.log" 2>&1 &
bridge_pid=$!
python3 source-nodes/command_guard --ros-args -p use_sim_time:=true >"$out/guard.log" 2>&1 &
guard_pid=$!
observer_pid=
if [ "${JANUS_VIDEO_DIRECT_CALLBACK:-0}" = 1 ]; then
  set -- "$@" --direct-observer-source source-nodes/qr_observer
else
  python3 scripts/limited_observer.py source-nodes/qr_observer --ros-args -p use_sim_time:=true >"$out/observer.log" 2>&1 &
  observer_pid=$!
fi
cleanup() {
  kill -TERM ${observer_pid:+"$observer_pid"} "$guard_pid" "$bridge_pid" "$gz_pid" 2>/dev/null || true
  wait ${observer_pid:+"$observer_pid"} "$guard_pid" "$bridge_pid" "$gz_pid" 2>/dev/null || true
}
trap cleanup EXIT
{
  uname -a
  cat /etc/os-release
  gz sim --versions
  python3 -c 'import cv2; print("opencv", cv2.__version__, "threads", cv2.getNumThreads())'
  env | grep -E '^(ROS_DOMAIN_ID|GZ_PARTITION|GZ_IP|ROS_AUTOMATIC_DISCOVERY_RANGE|OPENCV_FOR_THREADS_NUM|OPENBLAS_NUM_THREADS|OMP_NUM_THREADS|JANUS_VIDEO_DIRECT_CALLBACK|JANUS_VIDEO_REALTIME_FACTOR)='
} > "$out/environment.txt"
python3 scripts/live_replay.py fixtures "$out/recording" "$@" >"$out/driver.log" 2>&1 &
replay_pid=$!
sleep 3
ros2 topic info -v /cmd_vel > "$out/cmd-vel-graph.txt" 2>&1
if [ "${JANUS_VIDEO_DIRECT_CALLBACK:-0}" = 1 ]; then
  printf 'The retained RGB8 images use the pinned observer callback in process.\n' > "$out/image-graph.txt"
  ros2 topic info -v /janus/qr/observation > "$out/observation-graph.txt" 2>&1
else
  ros2 topic info -v /janus/camera/image_raw > "$out/image-graph.txt" 2>&1
fi
gz topic -l > "$out/gazebo-topics.txt"
wait "$replay_pid"
