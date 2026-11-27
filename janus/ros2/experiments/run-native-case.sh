#!/usr/bin/env bash
# Launch one isolated native scenario. Run from a sourced Jazzy overlay.
set -euo pipefail
if [[ $# -ne 2 ]]; then
  printf 'Usage: %s CASE NEW_OUTPUT_DIR\n' "$0" >&2
  exit 2
fi
case_id=$1
output=$2
mkdir -p "$(dirname "$output")"
mkdir "$output"
output=$(cd "$output" && pwd)
experiment_dir=$(cd "$(dirname "$0")" && pwd)
export ROS_DOMAIN_ID=${JANUS_ROS_DOMAIN_ID:-$((10 + $$ % 60))}
export ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}
export GZ_PARTITION="janus-${case_id}-$$"
export GZ_IP=${GZ_IP:-127.0.0.1}
export ROS_LOG_DIR="$output/roslog"
export LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-1}
export LP_NUM_THREADS=${LP_NUM_THREADS:-2}
world=$(ros2 pkg prefix --share arche_sim)/worlds/janus.sdf
launch_options=()
if [[ $case_id == Q* ]]; then
  launch_options+=("decoder_backend:=${JANUS_DECODER_BACKEND:-zbar}")
fi
case "$case_id" in
  M01|M02|O04|O05) launch_options+=("run_qr_observer:=false") ;;
  O08) launch_options+=("run_qr_observer:=false" "require_recovery_permission:=true") ;;
  Q01|Q02|Q03|Q04|Q05|Q06|Q07|Q13|Q17|Q19)
    target_u=$(python3 "$experiment_dir/make-world.py" "$case_id" "$world" "$output" "${JANUS_CAMERA_VARIANT:-nominal}")
    launch_options+=("target_u_px:=$target_u")
    world="$output/world.sdf"
    ;;
  Q08) launch_options+=("target_id:=JANUS-DOCK-AUTHORIZED") ;;
esac
if [[ $case_id == Q13 && ${JANUS_Q13_MODE:-resume} == regression ]]; then
  launch_options+=("camera_raw_topic:=/janus/camera/native_raw")
fi
if [[ $case_id == O05 && ${JANUS_O05_RECOVERY:-0} == 1 ]]; then
  launch_options+=("require_recovery_permission:=true" "allow_odom_regression_recovery:=true")
fi
if [[ $case_id == Q* ]]; then
  # This runner owns its explicit simulator fixture. Multi-marker worlds keep the full detector.
  is_single=$(python3 - "$world" <<'PY'
import sys
import xml.etree.ElementTree as ET
world = ET.parse(sys.argv[1])
uris = [node.text for node in world.findall('./world/include/uri')]
print('true' if uris == ['model://arche_janus', 'model://janus_dock'] else 'false')
PY
)
  launch_options+=("known_single_marker_fixture:=$is_single")
fi
if [[ ${JANUS_XVFB:-0} == 1 ]]; then
  launch_options+=("gz_flags:=-r -s -v 3")
  launch=(xvfb-run -a ros2 launch)
else
  export EGL_PLATFORM=${EGL_PLATFORM:-surfaceless}
  launch=(ros2 launch)
fi
launch+=(arche_bringup janus.launch.py "world_sdf:=$world" "${launch_options[@]}")
sha256sum "$world" > "$output/world-sha256.txt"
sha256sum "$experiment_dir/native_case.py" "$experiment_dir/run-native-case.sh" \
  "$experiment_dir/make-world.py" "$experiment_dir/validate-native.py" > "$output/test-code-sha256.txt"
printf '%q ' "${launch[@]}" > "$output/invocation.txt"
printf '\n' >> "$output/invocation.txt"
printf 'ROS_DOMAIN_ID=%s\nGZ_PARTITION=%s\n' "$ROS_DOMAIN_ID" "$GZ_PARTITION" > "$output/isolation.txt"
env --default-signal=INT,QUIT setsid --wait "${launch[@]}" > "$output/launch.log" 2>&1 &
launch_pid=$!
stop_group() {
  local group=$1
  if kill -0 -- "-$group" 2>/dev/null; then
    kill -INT "$group" 2>/dev/null || true
    for tick in $(seq 1 60); do
      kill -0 -- "-$group" 2>/dev/null || break
      sleep .1
    done
    kill -TERM -- "-$group" 2>/dev/null || true
    for tick in $(seq 1 20); do
      kill -0 -- "-$group" 2>/dev/null || break
      sleep .1
    done
    kill -KILL -- "-$group" 2>/dev/null || true
  fi
  wait "$group" || true
}
recorder_pid=
cleanup() {
  [[ -z $recorder_pid ]] || stop_group "$recorder_pid"
  stop_group "$launch_pid"
}
trap cleanup EXIT
if [[ ${JANUS_RECORD_BAG:-0} == 1 ]]; then
  env --default-signal=INT,QUIT setsid --wait ros2 bag record --use-sim-time --disable-keyboard-controls \
    -s mcap --storage-preset-profile zstd_fast -o "$output/bag" \
    --qos-profile-overrides-path "$(ros2 pkg prefix --share arche_sim)/config/bag_qos.yaml" \
    /clock /odom /joint_states /janus/ground_truth /janus/cmd_vel/request /cmd_vel \
    /janus/command_guard/status /janus/abort /janus/recovery/authorize \
    /janus/camera/image_raw /janus/camera/camera_info /janus/qr/observation > "$output/recorder.log" 2>&1 &
  recorder_pid=$!
fi
controller_options=()
if [[ -n ${JANUS_WALL_S:-} ]]; then
  controller_options+=(--wall-s "$JANUS_WALL_S")
fi
if [[ -n ${JANUS_CONTROL_DEPTH_M:-} ]]; then
  controller_options+=(--control-depth-m "$JANUS_CONTROL_DEPTH_M")
fi
if [[ $case_id == O05 && ${JANUS_O05_RECOVERY:-0} == 1 ]]; then
  controller_options+=(--o05-recovery)
fi
if [[ $case_id == Q13 ]]; then
  controller_options+=(--q13-mode "${JANUS_Q13_MODE:-resume}")
fi
if [[ $case_id == Q13 && ${JANUS_Q13_MODE:-resume} != regression ]]; then
  for try in $(seq 1 100); do
    camera_pid=$(sed -n 's/.*parameter_bridge-3.*process started with pid \[\([0-9][0-9]*\)\].*/\1/p' "$output/launch.log")
    [[ -z $camera_pid ]] || break
    kill -0 "$launch_pid"
    sleep .1
  done
  [[ -n $camera_pid ]] || { printf 'Camera bridge PID missing\n' >&2; exit 1; }
  controller_options+=(--camera-bridge-pid "$camera_pid")
fi
python3 "$experiment_dir/native_case.py" "$case_id" "$output" "${controller_options[@]}" | tee "$output/controller.log"
if [[ -n $recorder_pid ]]; then
  stop_group "$recorder_pid"
  recorder_pid=
  ros2 bag info "$output/bag" > "$output/bag-info.txt"
  ros2 run arche_experiments export_bag "$output/bag" "$output/bag-trace.jsonl"
fi
