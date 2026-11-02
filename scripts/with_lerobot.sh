#!/usr/bin/env bash
set -euo pipefail
unset PYTHONHOME
ARMHUB_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="$ARMHUB_ROOT/.venv_lerobot/bin/python"
if [[ ! -x "$PY" ]]; then
  echo '先运行 bash scripts/setup_lerobot.sh 安装独立 LeRobot 环境。' >&2
  exit 1
fi
# Python subprocesses (notably Rerun's viewer) must use this environment too.
export PATH="$ARMHUB_ROOT/.venv_lerobot/bin:$PATH"
export LD_LIBRARY_PATH="$ARMHUB_ROOT/.venv_lerobot/lib/python3.10/site-packages/cmeel.prefix/lib:$ARMHUB_ROOT/hardware_ws/build/sdk:$ARMHUB_ROOT/hardware_ws/build/sdk/third_party/lcm/lcm:$ARMHUB_ROOT/hardware_ws/build/sdk/third_party/serial_cmake:$ARMHUB_ROOT/vendor/panthera/sdk/deps/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$ARMHUB_ROOT/integrations/panthera_lerobot:$ARMHUB_ROOT/vendor/panthera/sdk/python:$ARMHUB_ROOT"
export HF_LEROBOT_HOME="$ARMHUB_ROOT/datasets/lerobot"
cd "$ARMHUB_ROOT"
exec "$PY" "$@"
