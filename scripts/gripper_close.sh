#!/usr/bin/env bash
set -euo pipefail
ARMHUB_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$ARMHUB_ROOT/scripts/with_hardware.sh" -m examples.gripper close "$@"
