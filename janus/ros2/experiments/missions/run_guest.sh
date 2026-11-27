#!/usr/bin/env bash
set -eo pipefail
usage="usage: run_guest.sh CASE NEW_OUTPUT_DIR ROS_OVERLAY_SETUP [WALL_LIMIT_SECONDS]"
[ "$#" -ge 3 ] || { printf '%s\n' "$usage"; exit 64; }
case_name=$1
out=$2
overlay=$3
wall_limit=${4:-125}
script_dir=$(cd -- "$(dirname -- "$0")" && pwd)
source /opt/ros/jazzy/setup.bash
source "$overlay"
export ROS_DOMAIN_ID=82 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION="arche_janus_missions_${case_name,,}" GZ_IP=127.0.0.1
export LIBGL_ALWAYS_SOFTWARE=1 EGL_PLATFORM=surfaceless LP_NUM_THREADS=2
export OPENCV_FOR_THREADS_NUM=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export PYTHONDONTWRITEBYTECODE=1
codec="${JANUS_CODEC_DIR:-$HOME/janus-mission-codec}"
export PYTHONPATH="$codec/extracted/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$codec/extracted/usr/lib/aarch64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
python3 -c 'import zxingcpp'
mkdir -- "$out"
out=$(realpath -- "$out")
export ROS_LOG_DIR="$out/roslog"
sim="$(ros2 pkg prefix arche_sim)/share/arche_sim"
python3 "$script_dir/make_world.py" "$case_name" "$sim" "$out"
world="janus_mission_${case_name,,}"
sed "s|/world/janus/clock|/world/$world/clock|" "$sim/config/bridge.yaml" > "$out/bridge.yaml"
if [ "$case_name" = O06 ]; then
    for wheel in left_wheel right_wheel caster_wheel; do
        cat >> "$out/bridge.yaml" <<YAML
- ros_topic_name: /janus/lab/floor_contact/$wheel
  gz_topic_name: /janus/gz/floor_contact/$wheel
  ros_type_name: ros_gz_interfaces/msg/Contacts
  gz_type_name: gz.msgs.Contacts
  direction: GZ_TO_ROS
YAML
    done
fi
{
    uname -a
    cat /etc/os-release
    dpkg-query -W ros-jazzy-ros-base ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim \
        ros-jazzy-gz-sim-vendor ros-jazzy-gz-physics-vendor ros-jazzy-gz-dartsim-vendor \
        python3-opencv libgl1-mesa-dri
    gz sim --versions
    dpkg-deb -f "$codec/python3-zxing-cpp_2.2.1-3_arm64.deb" Package Version Architecture
    sha256sum "$codec/"*.deb
    findmnt -T "$out" -o TARGET,SOURCE,FSTYPE,OPTIONS
    printf 'ROS_DOMAIN_ID=%s\nGZ_PARTITION=%s\nROS_AUTOMATIC_DISCOVERY_RANGE=%s\n' \
        "$ROS_DOMAIN_ID" "$GZ_PARTITION" "$ROS_AUTOMATIC_DISCOVERY_RANGE"
    python3 -c 'import cv2, platform; print(platform.python_version()); print(cv2.__version__)'
} > "$out/environment.txt"
sha256sum "$out/world.sdf" "$out/world.json" "$out/bridge.yaml" "$sim/config/camera_bridge.yaml" \
    "$sim/models/arche_janus/model.sdf" "$sim/models/janus_dock/model.sdf" \
    "$sim/models/janus_dock/meshes/marker.dae" \
    "$sim/models/janus_dock/meshes/marker.png" \
    "$(ros2 pkg prefix arche_experiments)/lib/arche_experiments/command_guard" \
    "$(ros2 pkg prefix arche_experiments)/lib/arche_experiments/qr_observer" \
    "$(ros2 pkg prefix gz_physics_vendor)/opt/gz_physics_vendor/lib/gz-physics-7/engine-plugins/libgz-physics-dartsim-plugin.so" \
    "$script_dir"/*.py "$script_dir/run_guest.sh" > "$out/sha256.txt"
if [ -f "$out/loaded-arche.sdf" ]; then
    sha256sum "$out/loaded-arche.sdf" >> "$out/sha256.txt"
fi
printf '%q ' ros2 launch "$script_dir/launch_mission.py" "world_sdf:=$out/world.sdf" "bridge:=$out/bridge.yaml" > "$out/commands.sh"
printf '\n' >> "$out/commands.sh"
printf '%q ' python3 "$script_dir/live_mission.py" "$case_name" "$out" --wall-s "$wall_limit" >> "$out/commands.sh"
printf '\n' >> "$out/commands.sh"
setsid ros2 launch "$script_dir/launch_mission.py" \
    "world_sdf:=$out/world.sdf" "bridge:=$out/bridge.yaml" > "$out/launch.log" 2>&1 &
launch_pid=$!
stop_launch() {
    kill -INT "$launch_pid" 2>/dev/null || true
    for (( attempt=0; attempt<50; attempt++ )); do
        if ! kill -0 -- "-$launch_pid" 2>/dev/null; then
            break
        fi
        sleep .2
    done
    kill -TERM -- "-$launch_pid" 2>/dev/null || true
    dir="$out"
    wait "$launch_pid" 2>/dev/null || true
    sha256sum "$dir/environment.txt" "$dir/commands.sh" >> "$dir/sha256.txt"
}
trap stop_launch EXIT
set +e
python3 "$script_dir/live_mission.py" "$case_name" "$out" --wall-s "$wall_limit" > "$out/mission.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$out/mission-exit.txt"
exit "$rc"
