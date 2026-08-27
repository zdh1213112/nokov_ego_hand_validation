#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
VENV_DIR="${PROJECT_DIR}/.venv-sync"

command -v python3 >/dev/null 2>&1 || {
  echo "error: python3 is required" >&2
  exit 2
}

if ! python3 -c 'import ensurepip' >/dev/null 2>&1; then
  PY_MINOR="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
  echo "error: Python venv/ensurepip is not installed" >&2
  echo "Ubuntu/Debian: sudo apt install python3-venv python${PY_MINOR}-venv" >&2
  exit 2
fi

python3 -m venv "${VENV_DIR}"
"${VENV_DIR}/bin/python" -m pip install --upgrade pip
"${VENV_DIR}/bin/python" -m pip install -r "${SCRIPT_DIR}/requirements-calibration.txt"
"${VENV_DIR}/bin/python" "${SCRIPT_DIR}/test_sync_core.py"
"${VENV_DIR}/bin/python" "${SCRIPT_DIR}/test_spatial_calibration_core.py"

echo
echo "Linux synchronization environment is ready: ${VENV_DIR}"
echo "Run tools/run_ego_nokov_alignment.py to process time and spatial alignment."
