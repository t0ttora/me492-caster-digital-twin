#!/usr/bin/env bash
set -eo pipefail
lane=/mnt/janus-evidence/gazebo-geometry
source_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
case_id=$1
variant=$2
run_id=$3
[[ $case_id =~ ^(M07|M11|M12|O02|O03|Q14)$ && $run_id =~ ^[a-zA-Z0-9_-]+$ ]]
[[ $run_id == "$case_id"-gazebo-* ]]
[[ $(findmnt -no FSTYPE --target "$lane") == virtiofs ]]
exec 9>"$lane/domain-84.lock"
flock -n 9 || { echo "ROS domain 84 already has a geometry run"; exit 1; }
out=$lane/runs/$run_id
mkdir "$out"
mkdir "$out"/{roslog,cache,tmp}
source /opt/ros/jazzy/setup.bash
source "$HOME/janus-build/install/setup.bash"
export ROS_DOMAIN_ID=84 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION="janus_geometry_84_${run_id}"
export ROS_LOG_DIR="$out/roslog" TMPDIR="$out/tmp" XDG_CACHE_HOME="$out/cache"
export GZ_SIM_RESOURCE_PATH="$lane/arche_sim/models" GZ_IP=127.0.0.1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 LP_NUM_THREADS=2 OPENCV_FOR_THREADS_NUM=1 PYTHONDONTWRITEBYTECODE=1
cd "$out"
sha256sum "$source_dir/"*.py "$source_dir/run_guest.sh" "$lane/arche_sim/models/arche_janus/model.sdf" \
  "$lane/arche_sim/worlds/janus.sdf" "$lane/arche_sim/config/bridge.yaml" > input-sha256.txt
integration="$lane/integration-629bf31"
export PYTHONPATH="$integration:$PYTHONPATH"
sha256sum "$integration/"{command_guard,qr_observer} "$integration/arche_experiments/"*.py >> input-sha256.txt
if [[ $case_id == M12 || $case_id == O03 || $case_id == Q14 ]]; then
  JANUS_NATIVE_CONTACT_PLUGIN="$lane/instrumentation-v3/build/libjanus_geometry_contact.so"
  export JANUS_NATIVE_CONTACT_PLUGIN
  sha256sum "$JANUS_NATIVE_CONTACT_PLUGIN" >> input-sha256.txt
fi
if [[ $case_id == O03 || $case_id == Q14 ]]; then
  JANUS_NATIVE_MOVER_PLUGIN="$lane/instrumentation-v3/build/libjanus_geometry_mover.so"
  export JANUS_NATIVE_MOVER_PLUGIN
  sha256sum "$JANUS_NATIVE_MOVER_PLUGIN" >> input-sha256.txt
fi
if [[ $case_id == Q14 ]]; then
  sha256sum "$lane/arche_sim/models/janus_dock/meshes/"marker.{dae,png} >> input-sha256.txt
fi
{
  uname -m
  cat /etc/os-release
  gz sim --versions
  printenv ROS_DISTRO ROS_DOMAIN_ID GZ_PARTITION
  command -v ros2 gz
  ros2 pkg prefix arche_experiments
  dpkg-query -W 'ros-jazzy-ros-gz*' 'ros-jazzy-gz-*vendor'
  findmnt --target "$out"
} > versions.txt
cp "$lane/arche_sim/config/robot_config.json" robot_config.json
python3 -B "$source_dir/scene.py" "$lane/arche_sim" "$out" "$case_id" "$variant"
gz sdf -k "$out/scene.sdf" > sdf_check.log 2>&1
pids=()
cleanup() {
  for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
start() {
  local log=$1
  shift
  setsid "$@" > "$out/$log" 2>&1 &
  pids+=("$!")
}
start contacts.log python3 -B "$source_dir/save_contacts.py" "$out"
start gaz.log gz sim -r -s --headless-rendering --seed 492 --physics-engine gz-physics-dartsim-plugin -v 3 "$out/scene.sdf"
start bridge.log ros2 run ros_gz_bridge parameter_bridge --ros-args -p "config_file:=$out/bridge.yaml"
start guard.log python3 -B "$integration/command_guard" --ros-args -p use_sim_time:=true
if [[ $case_id == Q14 || $case_id == O03 ]]; then
  cp "$lane/arche_sim/config/camera_bridge.yaml" camera_bridge.yaml
  start camera_bridge.log ros2 run ros_gz_bridge parameter_bridge --ros-args -p "config_file:=$out/camera_bridge.yaml" -p override_frame_id:=camera_optical_frame
  start injection.log python3 -B "$source_dir/inject.py" "$case_id" "$variant"
fi
if [[ $case_id == Q14 ]]; then
  case "$variant" in
    qr-runtime*) observer="$lane/runtime-snapshot-0358" ;;
    qr-recovery*) observer="$lane/qr-recovery" ;;
    *) observer="$integration" ;;
  esac
  cp "$observer/provenance.txt" qr_observer_provenance.txt
  sha256sum "$observer/qr_observer" "$observer/arche_experiments/"*.py >> input-sha256.txt
  start qr.log env PYTHONPATH="$observer:$PYTHONPATH" python3 -B "$observer/qr_observer" --ros-args -p use_sim_time:=true
fi
wall_budget=210s
if [[ $case_id == Q14 ]]; then
  wall_budget=310s
fi
timeout --signal=INT --kill-after=8s "$wall_budget" python3 -B "$source_dir/drive.py" "$out" > controller.log 2>&1
