#!/usr/bin/env bash
set -eo pipefail
ARMHUB_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
set -u
cd "$ARMHUB_ROOT"
if [[ ! -f vendor/panthera/sdk/bindings.cpp ]]; then
  python3 scripts/import_panthera_sdk.py
fi
SDK_ROOT="$ARMHUB_ROOT/vendor/panthera/sdk"
cmake -S hardware_ws -B hardware_ws/build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$SDK_ROOT/deps/usr;/opt/ros/humble" \
  -DSERIALPORT_INCLUDE_DIR="$SDK_ROOT/deps/usr/include" \
  -DSERIALPORT_LIBRARY="$SDK_ROOT/deps/usr/lib/x86_64-linux-gnu/libserialport.so" \
  -Dpybind11_DIR="$SDK_ROOT/build_deps/pybind11/share/cmake/pybind11" \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
cmake --build hardware_ws/build --target _hightorque_robot -j 4
