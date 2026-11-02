#!/usr/bin/env bash
set -euo pipefail
ARMHUB_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$ARMHUB_ROOT/scripts/with_dynamics.sh" -m examples.collect_j6_gravity "$@"
