#!/usr/bin/env bash
set -eo pipefail
ARMHUB_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
set -u
SDK_ROOT="$ARMHUB_ROOT/vendor/panthera/sdk"
if [[ ! -f "$SDK_ROOT/python/hightorque_robot/_hightorque_robot.cpython-310-x86_64-linux-gnu.so" ]]; then
  echo '先运行 bash scripts/build_hardware.sh 编译官方 SDK。' >&2
  exit 1
fi
export LD_LIBRARY_PATH="$ARMHUB_ROOT/hardware_ws/build/sdk:$ARMHUB_ROOT/hardware_ws/build/sdk/third_party/lcm/lcm:$ARMHUB_ROOT/hardware_ws/build/sdk/third_party/serial_cmake:$SDK_ROOT/deps/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$ARMHUB_ROOT:$SDK_ROOT/python:${PYTHONPATH:-}"
cd "$ARMHUB_ROOT"
exec "${ARMHUB_HARDWARE_PYTHON:-/usr/bin/python3}" "$@"
