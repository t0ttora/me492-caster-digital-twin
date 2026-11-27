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
case "$case_id" in
  M01|M02|O04|O08) launch_options+=("run_qr_observer:=false") ;;
  Q02|Q03)
    lateral=0.28
    [[ $case_id == Q02 ]] || lateral=-0.28
    python3 - "$world" "$output/world.sdf" "$lateral" <<'PY'
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
source, dest, offset = sys.argv[1:]
world = ET.parse(source)
include = next(row for row in world.findall('./world/include')
               if row.findtext('uri') == 'model://janus_dock')
pose = include.find('pose')
parts = pose.text.split()
parts[1] = offset
pose.text = ' '.join(parts)
world.write(Path(dest), encoding='utf-8', xml_declaration=True)
PY
    world="$output/world.sdf"
    ;;
  Q08) launch_options+=("target_id:=JANUS-DOCK-AUTHORIZED") ;;
esac
if [[ ${JANUS_XVFB:-0} == 1 ]]; then
  launch_options+=("gz_flags:=-r -s -v 3")
  launch=(xvfb-run -a ros2 launch)
else
  export EGL_PLATFORM=${EGL_PLATFORM:-surfaceless}
  launch=(ros2 launch)
fi
launch+=(arche_bringup janus.launch.py "world_sdf:=$world" "${launch_options[@]}")
sha256sum "$world" > "$output/world-sha256.txt"
printf '%q ' "${launch[@]}" > "$output/invocation.txt"
printf '\n' >> "$output/invocation.txt"
printf 'ROS_DOMAIN_ID=%s\nGZ_PARTITION=%s\n' "$ROS_DOMAIN_ID" "$GZ_PARTITION" > "$output/isolation.txt"
env --default-signal=INT,QUIT setsid --wait "${launch[@]}" > "$output/launch.log" 2>&1 &
launch_pid=$!
cleanup() {
  if kill -0 -- "-$launch_pid" 2>/dev/null; then
    kill -INT "$launch_pid" 2>/dev/null || true
    for tick in $(seq 1 60); do
      kill -0 -- "-$launch_pid" 2>/dev/null || break
      sleep .1
    done
    kill -TERM -- "-$launch_pid" 2>/dev/null || true
    for tick in $(seq 1 20); do
      kill -0 -- "-$launch_pid" 2>/dev/null || break
      sleep .1
    done
    kill -KILL -- "-$launch_pid" 2>/dev/null || true
  fi
  wait "$launch_pid" || true
}
trap cleanup EXIT
controller_options=()
if [[ $case_id == Q13 ]]; then
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
