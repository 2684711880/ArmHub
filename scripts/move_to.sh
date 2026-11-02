#!/usr/bin/env bash
set -euo pipefail
ARMHUB_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
args=("$@")
backend=simulation
for ((i=0; i<${#args[@]}; i++)); do
  if [[ "${args[i]}" == --backend && $((i+1)) -lt ${#args[@]} ]]; then
    backend="${args[i+1]}"
  elif [[ "${args[i]}" == --backend=* ]]; then
    backend="${args[i]#--backend=}"
  fi
done
if [[ "$backend" == hardware ]]; then
  exec bash "$ARMHUB_ROOT/scripts/with_dynamics.sh" -m examples.move_to "$@"
fi
exec bash "$ARMHUB_ROOT/scripts/with_ros.sh" python3 -m examples.move_to "$@"
