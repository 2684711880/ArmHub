#!/usr/bin/env bash
set -euo pipefail
ARMHUB_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bash "$ARMHUB_ROOT/scripts/setup_dynamics.sh"
"$ARMHUB_ROOT/.venv_dynamics/bin/python" -m pip install -r "$ARMHUB_ROOT/deps/vision-requirements.txt"
# This venv inherits Ubuntu's python3-opencv / python3-yaml.
"$ARMHUB_ROOT/.venv_dynamics/bin/python" -c 'import cv2, yaml, pyrealsense2; print("Vision dependencies ready; OpenCV", cv2.__version__)'
