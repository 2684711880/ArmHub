#!/usr/bin/env bash
set -eo pipefail
ARMHUB_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ARMHUB_ROOT/ros_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID=88
export ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11488
export GAZEBO_MODEL_DATABASE_URI=''
export GAZEBO_RESOURCE_PATH="/usr/share/gazebo-11:${GAZEBO_RESOURCE_PATH:-}"
export PYTHONPATH="$ARMHUB_ROOT:${PYTHONPATH:-}"
cd "$ARMHUB_ROOT"
exec "$@"
