#!/usr/bin/env bash
# One command on The Construct, after this folder is unzipped.
# Builds the C++ dual-antenna monitor, replays four simulated flights through it over ROS 2
# topics, compares its alarms with the Python tests, then counts a real UAV GPS file.
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

if ! command -v colcon >/dev/null 2>&1; then
  if sudo -n true 2>/dev/null; then
    sudo apt-get update
    sudo apt-get install -y python3-colcon-common-extensions
  else
    echo "FAILED"
    echo "colcon is not installed."
    exit 1
  fi
fi

cd "$ROOT/ros2"
colcon build --packages-select gim_monitor
# shellcheck disable=SC1091
source "$ROOT/ros2/install/setup.bash"

# A monitor left over from an earlier run would answer on the same topics.
pkill -f "lib/gim_monitor/monitor" >/dev/null 2>&1 || true

# Each recorded flight goes to a fresh copy of the C++ monitor, started by the check itself.
python3 "$ROOT/ros2/check_monitor.py"
STATUS=$?
if [[ "$STATUS" -ne 0 ]]; then
  exit "$STATUS"
fi
python3 "$ROOT/ros2/replay_uav.py"
exit $?
