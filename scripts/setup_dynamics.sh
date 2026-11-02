#!/usr/bin/env bash
set -euo pipefail
ARMHUB_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
/usr/bin/python3 -m venv --system-site-packages "$ARMHUB_ROOT/.venv_dynamics"
"$ARMHUB_ROOT/.venv_dynamics/bin/python" -m pip install -r "$ARMHUB_ROOT/deps/gravity-requirements.txt"
