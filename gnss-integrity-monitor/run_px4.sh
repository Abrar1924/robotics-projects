#!/usr/bin/env bash
# Build PX4 v1.14.3 on a Humble rosject, start it with no real drone, and check the C++ monitor
# against the GPS course and yaw PX4 reports.
# The first run can take about 20 minutes. Later runs reuse the build in ~/PX4-Autopilot.
set -o pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"
PX4="${PX4_ROOT:-$HOME/PX4-Autopilot}"

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

if ! python3 -c "import pymavlink" >/dev/null 2>&1; then
  if sudo -n true 2>/dev/null; then
    sudo apt-get update
    sudo apt-get install -y python3-pymavlink || pip3 install --user pymavlink
  else
    pip3 install --user pymavlink
  fi
fi
if ! python3 -c "import pymavlink" >/dev/null 2>&1; then
  echo "FAILED"
  echo "pymavlink is not installed."
  exit 1
fi

cd "$ROOT/ros2"
colcon build --packages-select gim_monitor
# shellcheck disable=SC1091
source "$ROOT/ros2/install/setup.bash"

if [[ ! -x "$PX4/build/px4_sitl_default/bin/px4" ]]; then
  echo "Building PX4 v1.14.3. The first run can take about 20 minutes."
  if [[ ! -d "$PX4/.git" ]]; then
    git clone --branch v1.14.3 --depth 1 https://github.com/PX4/PX4-Autopilot.git "$PX4"
  fi
  git -C "$PX4" submodule update --init --recursive
  if ! command -v cmake >/dev/null 2>&1 || ! command -v ninja >/dev/null 2>&1; then
    # ubuntu.sh stops early on this tag: requirements.txt says matplotlib>=3.0.*
    # and current pip rejects that line, so the packages below are installed directly.
    (cd "$PX4" && DEBIAN_FRONTEND=noninteractive bash Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools) || true
  fi
  python3 -m pip install --user \
    'empy==3.3.4' \
    kconfiglib \
    jinja2 \
    jsonschema \
    'numpy>=1.13' \
    packaging \
    pyros-genmsg \
    pyyaml \
    pyserial \
    toml \
    cerberus \
    future \
    six \
    wheel \
    'nunavut>=1.1.0' \
    pymavlink
  export PATH="$HOME/.local/bin:$PATH"
  if ! python3 -c "import kconfiglib, menuconfig, em"; then
    echo "FAILED"
    echo "PX4's Python packages did not install."
    exit 1
  fi
  rm -rf "$PX4/build/px4_sitl_default"
  # CMake 4 refuses projects that ask for a minimum below 3.5, as parts of this PX4 tag do.
  # Older CMake ignores this variable.
  export CMAKE_POLICY_VERSION_MINIMUM=3.5
  (cd "$PX4" && make px4_sitl_default)
fi
if [[ ! -x "$PX4/build/px4_sitl_default/bin/px4" ]]; then
  echo "FAILED"
  echo "PX4 did not build. The log above is the compiler output."
  exit 1
fi

# killall matches the program name. A pattern search would also match this script.
killall -9 px4 >/dev/null 2>&1 || true
# A monitor left over from an earlier run would answer on the same topics.
pkill -f "lib/gim_monitor/monitor" >/dev/null 2>&1 || true
sleep 1

SUP_PID=""
cleanup() {
  if [[ -n "$SUP_PID" ]]; then kill "$SUP_PID" 2>/dev/null || true; fi
  # "ros2 run" starts the node as a child: stop the node itself as well.
  pkill -f "lib/gim_monitor/monitor" >/dev/null 2>&1 || true
  killall -9 px4 >/dev/null 2>&1 || true
}
trap cleanup EXIT

ros2 run gim_monitor monitor >/tmp/gim-monitor.log 2>&1 &
SUP_PID=$!

export PX4_ROOT="$PX4"
python3 "$ROOT/ros2/px4_check.py"
exit $?
