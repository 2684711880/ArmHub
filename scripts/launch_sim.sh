#!/usr/bin/env bash
set -euo pipefail
ARMHUB_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
robot="${1:-panthera_ht}"
if [[ $# -gt 0 ]]; then shift; fi
case "$robot" in panthera_ht|nero7) ;; *) echo '机型必须为 panthera_ht / nero7' >&2; exit 2;; esac
exec bash "$ARMHUB_ROOT/scripts/with_ros.sh" ros2 launch armhub_sim sim.launch.py "robot:=$robot" "$@"
