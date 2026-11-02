#!/usr/bin/env bash
set -euo pipefail
unset PYTHONPATH PYTHONHOME
ARMHUB_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ARMHUB_ROOT"
/usr/bin/python3 integrations/panthera_lerobot/prepare_sources.py
[[ -x .venv_lerobot/bin/python ]] || /usr/bin/python3 -m venv .venv_lerobot
PY="$ARMHUB_ROOT/.venv_lerobot/bin/python"
"$PY" -m pip install --upgrade pip
# CPU wheels keep initial setup independent of the workstation's CUDA version.
# A CUDA training environment can replace these with the matching GPU wheels.
if "$PY" -c 'import importlib.metadata as m; assert m.version("pin") == "2.7.0"' 2>/dev/null; then
  "$PY" -m pip uninstall -y pin hpp-fcl eigenpy
fi
"$PY" -m pip install --index-url https://download.pytorch.org/whl/cpu 'torch==2.7.1' 'torchvision==0.22.1'
"$PY" -m pip install 'numpy==2.2.6' 'scipy==1.15.3' 'pin==3.8.0' 'cmeel-urdfdom==4.0.1' 'cmeel-tinyxml2==10.0.0' 'pyrealsense2==2.57.7.10387' 'torchcodec==0.5.0' 'opencv-python-headless<4.12' 'PyQt5==5.15.11' 'ruckig==0.12.2' 'matplotlib==3.9.4'
"$PY" -m pip install -e integrations/panthera_lerobot/Panthera-HT_lerobot/lerobot -e integrations/panthera_lerobot/Panthera-HT_lerobot/Panthera-HT_robot_lerobot -c integrations/panthera_lerobot/constraints.txt
bash scripts/lerobot.sh check
