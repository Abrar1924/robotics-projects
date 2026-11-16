#!/usr/bin/env bash
# Nav2 follows a straight path in the Gazebo yard. The C++ program limits the speed.
set -o pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "FAILED"
  echo "ROS 2 Humble is not on this computer. Open a Humble rosject."
  exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

install_pkg() {
  if sudo -n true 2>/dev/null; then
    sudo apt-get update
    sudo apt-get install -y "$@"
  else
    echo "FAILED"
    echo "Missing $* and this session cannot install packages."
    exit 1
  fi
}

if ! command -v colcon >/dev/null 2>&1; then
  install_pkg python3-colcon-common-extensions
fi
if ! ros2 pkg prefix gazebo_ros >/dev/null 2>&1; then
  install_pkg ros-humble-gazebo-ros-pkgs
fi
if ! ros2 pkg prefix nav2_controller >/dev/null 2>&1; then
  install_pkg ros-humble-nav2-controller ros-humble-nav2-regulated-pure-pursuit-controller
fi
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

cd "$ROOT/ros2"
colcon build --packages-select mower_supervisor_cpp
# shellcheck disable=SC1091
source "$ROOT/ros2/install/setup.bash"

killall -9 gzserver gzmaster gzclient 2>/dev/null || true
pkill -9 -f controller_server 2>/dev/null || true
pkill -9 -f mower_supervisor_cpp/supervisor 2>/dev/null || true
sleep 1

export GAZEBO_MODEL_PATH="/usr/share/gazebo-11/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export MOWER_ROOT="$ROOT"
export PYTHONPATH="$ROOT:$ROOT/ros2/mower_supervisor${PYTHONPATH:+:$PYTHONPATH}"

GZ_PID=""
SUP_PID=""
NAV_PID=""
cleanup() {
  if [[ -n "$NAV_PID" ]]; then kill "$NAV_PID" 2>/dev/null || true; fi
  if [[ -n "$SUP_PID" ]]; then kill "$SUP_PID" 2>/dev/null || true; fi
  if [[ -n "$GZ_PID" ]]; then kill "$GZ_PID" 2>/dev/null || true; fi
  pkill -x gzserver 2>/dev/null || true
  pkill -f controller_server 2>/dev/null || true
}
trap cleanup EXIT

gzserver --verbose \
  -s libgazebo_ros_init.so \
  "$ROOT/ros2/mower_yard/worlds/yard.world" >/tmp/mower-gazebo.log 2>&1 &
GZ_PID=$!

ros2 run mower_supervisor_cpp supervisor &
SUP_PID=$!

ros2 run nav2_controller controller_server --ros-args \
  --params-file "$ROOT/ros2/mower_yard/nav2_params.yaml" \
  >/tmp/mower-nav2.log 2>&1 &
NAV_PID=$!

python3 "$ROOT/ros2/mower_yard/nav2_drive.py"
STATUS=$?
if [[ "$STATUS" -ne 0 ]]; then
  echo "--- last lines of /tmp/mower-nav2.log ---"
  tail -n 40 /tmp/mower-nav2.log || true
  echo "--- last lines of /tmp/mower-gazebo.log ---"
  tail -n 20 /tmp/mower-gazebo.log || true
fi
exit "$STATUS"
