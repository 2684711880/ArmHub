#!/usr/bin/env bash
set -euo pipefail
ARMHUB_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -x "$ARMHUB_ROOT/.venv_dynamics/bin/python" ]]; then
  echo '先运行 bash scripts/setup_dynamics.sh 安装独立动力学依赖。' >&2
  exit 1
fi
export ARMHUB_HARDWARE_PYTHON="$ARMHUB_ROOT/.venv_dynamics/bin/python"
export LD_LIBRARY_PATH="$ARMHUB_ROOT/.venv_dynamics/lib/python3.10/site-packages/cmeel.prefix/lib:${LD_LIBRARY_PATH:-}"
exec bash "$ARMHUB_ROOT/scripts/with_hardware.sh" "$@"
