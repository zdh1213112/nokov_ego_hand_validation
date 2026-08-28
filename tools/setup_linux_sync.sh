#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ENV_NAME="${NOKOV_CONDA_ENV:-nokov-ego-validation}"
ENV_FILE="${PROJECT_DIR}/environment.yml"

command -v conda >/dev/null 2>&1 || {
  echo "error: Conda is required" >&2
  exit 2
}

if conda env list | awk 'NF && $1 !~ /^#/ {print $1}' | grep -Fxq "${ENV_NAME}"; then
  conda env update --name "${ENV_NAME}" --file "${ENV_FILE}"
else
  conda env create --name "${ENV_NAME}" --file "${ENV_FILE}"
fi
conda run --no-capture-output -n "${ENV_NAME}" \
  python -m pip install -r "${SCRIPT_DIR}/requirements-calibration.txt"
conda run --no-capture-output -n "${ENV_NAME}" python "${SCRIPT_DIR}/test_sync_core.py"
conda run --no-capture-output -n "${ENV_NAME}" python "${SCRIPT_DIR}/test_spatial_calibration_core.py"

echo "Linux synchronization Conda environment is ready: ${ENV_NAME}"
