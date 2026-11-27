#!/usr/bin/env bash
# One supplemental Q04 fixture in the existing Ubuntu Jazzy simulator.
set -eo pipefail
[[ $# -ge 2 ]] || { echo "usage: $0 INPUT_DIR NEW_OUTPUT_DIR [WALL_SECONDS]" >&2; exit 64; }
inputs=$(realpath "$1")
out=$2
wall_seconds=${3:-130}
source /opt/ros/jazzy/setup.bash
source "${JANUS_ROS_OVERLAY:-$HOME/janus-build/install/setup.bash}"
script=$(realpath "$(dirname "$0")")
mkdir -- "$out"
out=$(realpath "$out")
cp -- "$inputs/fixture.json" "$inputs/world.sdf" "$out/"
export ROS_DOMAIN_ID=87 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION="janus-q04-mechanical-$$" GZ_IP=127.0.0.1
export ROS_LOG_DIR="$out/roslog"
export LIBGL_ALWAYS_SOFTWARE=1 EGL_PLATFORM=surfaceless LP_NUM_THREADS=2
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 OPENCV_FOR_THREADS_NUM=1
export PYTHONDONTWRITEBYTECODE=1
launch=(ros2 launch arche_bringup janus.launch.py "world_sdf:=$out/world.sdf"
        "decoder_backend:=zbar" "target_u_px:=320.0" "known_single_marker_fixture:=true")
python=(python3 "$script/stage_live.py" "$out" --wall-s "$wall_seconds")
{
    cat /etc/os-release
    uname -a
    gz sim --versions
    dpkg-query -W ros-jazzy-ros-base ros-jazzy-ros-gz-bridge ros-jazzy-gz-sim-vendor
    findmnt -T "$out" -o TARGET,SOURCE,FSTYPE
    printf 'ROS_DOMAIN_ID=%s\nGZ_PARTITION=%s\nROS_AUTOMATIC_DISCOVERY_RANGE=%s\n' \
        "$ROS_DOMAIN_ID" "$GZ_PARTITION" "$ROS_AUTOMATIC_DISCOVERY_RANGE"
} > "$out/environment.txt"
sim=$(ros2 pkg prefix --share arche_sim)
experiments=$(ros2 pkg prefix arche_experiments)
bringup=$(ros2 pkg prefix --share arche_bringup)
sha256sum "$out/world.sdf" "$out/fixture.json" "$script"/{stage_live.py,verify.py,run_guest.sh} \
    "$sim/models/arche_janus/model.sdf" "$sim/models/janus_dock/model.sdf" \
    "$sim/models/janus_dock/meshes/"{marker.dae,marker.png} "$sim/config/"{bridge.yaml,camera_bridge.yaml} \
    "$bringup/launch/janus.launch.py" "$experiments/lib/arche_experiments/"{command_guard,qr_observer} \
    > "$out/input-sha256.txt"
{
    printf '%q ' "${launch[@]}"
    printf '\n'
    printf '%q ' "${python[@]}"
    printf '\n'
} > "$out/invocation.txt"

env --default-signal=INT,QUIT setsid --wait "${launch[@]}" > "$out/launch.log" 2>&1 &
launch_pid=$!
stop_launch() {
    if kill -0 -- "-$launch_pid" 2>/dev/null; then
        kill -INT "$launch_pid" 2>/dev/null || true
        for tick in $(seq 1 30); do
            kill -0 -- "-$launch_pid" 2>/dev/null || break
            sleep .15
        done
        kill -TERM -- "-$launch_pid" 2>/dev/null || true
    fi
    wait "$launch_pid" 2>/dev/null || true
}
trap stop_launch EXIT
set +e
"${python[@]}" > "$out/controller.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$out/controller-exit.txt"
exit "$rc"
