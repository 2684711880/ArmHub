#!/usr/bin/env bash
set -euo pipefail
ARMHUB_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" == "gravity" || "${1:-}" == "home" ]]; then
  exec bash "$ARMHUB_ROOT/scripts/with_dynamics.sh" -m examples.hardware "$@"
fi
exec bash "$ARMHUB_ROOT/scripts/with_hardware.sh" -m examples.hardware "$@"
