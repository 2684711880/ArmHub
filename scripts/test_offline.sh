#!/usr/bin/env bash
# Offline only: no hardware sessions or camera capture.
set -euo pipefail
ARMHUB_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ARMHUB_ROOT"
case "${1:-core}" in
  core)
    python3 -m unittest discover -s tests -p 'test_contract.py' -v
    python3 -m unittest discover -s tests -p 'test_models.py' -v
    ;;
  full)
    exec bash scripts/with_lerobot.sh -m unittest discover -s tests -v
    ;;
  *)
    echo 'Usage: bash scripts/test_offline.sh [core|full]' >&2
    exit 2
    ;;
esac
