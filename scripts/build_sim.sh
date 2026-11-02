#!/usr/bin/env bash
set -eo pipefail
ARMHUB_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
set -u
cd "$ARMHUB_ROOT"
if [[ ! -f vendor/agilex/nero/robot_with_camera.urdf ]]; then
  python3 scripts/import_models.py
fi
python3 scripts/prepare_sim_models.py
cd ros_ws
colcon build --packages-select armhub_sim --event-handlers console_direct+
