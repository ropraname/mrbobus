#!/usr/bin/env bash
# Replay a short L2 bag in an isolated ROS domain; no robot commands or driver.
set -eo pipefail
if [[ $# -lt 2 ]]; then
  echo "Usage: $0 LIO_WORKSPACE BAG [PARAMS_YAML] [SECONDS=30]" >&2
  exit 2
fi
lio_ws=$1
bag_path=$2
config=${3:-"$lio_ws/install/mrbobus_lio/share/mrbobus_lio/config/l2_pi.yaml"}
seconds=${4:-30}
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source /opt/ros/jazzy/setup.bash
source "$lio_ws/install/setup.bash"
export ROS_DOMAIN_ID=44 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset CYCLONEDDS_URI ROS_LOCALHOST_ONLY
run_dir=$(mktemp -d /tmp/mrbobus-lio-replay.XXXXXX)
launch_pid= bag_pid=
cleanup() {
  [[ -z "$bag_pid" ]] || kill -TERM "$bag_pid" 2>/dev/null || true
  # Background Python launch may inherit SIGINT ignored; stop its owned node first.
  [[ -z "$launch_pid" ]] || pkill -INT -P "$launch_pid" 2>/dev/null || true
  [[ -z "$launch_pid" ]] || kill -TERM "$launch_pid" 2>/dev/null || true
  [[ -z "$bag_pid" ]] || wait "$bag_pid" 2>/dev/null || true
  [[ -z "$launch_pid" ]] || wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT
ros2 launch mrbobus_lio l2_lio.launch.py config:="$config" >"$run_dir/lio.log" 2>&1 &
launch_pid=$!
sleep 1
ros2 bag play "$bag_path" >"$run_dir/bag.log" 2>&1 &
bag_pid=$!
# Wall-time age is not meaningful on recorded timestamps; inspect pose/frequency.
python3 "$script_dir/check_lio.py" --seconds "$seconds" | tee "$run_dir/check.json"
echo "Replay logs: $run_dir"
