#!/usr/bin/env bash
set -euo pipefail
ARMHUB_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$ARMHUB_ROOT/scripts/movel.sh" --plane xy --trajectories square --direction fwd --size .04 --pitch .01 --speed .02 --square-from-current "$@"
