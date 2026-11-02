#!/usr/bin/env bash
set -euo pipefail
ARMHUB_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "$ARMHUB_ROOT/scripts/stop_sim.py"
