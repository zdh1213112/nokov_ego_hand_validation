#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
SDK_WHEEL="${1:-${PROJECT_DIR}/vendor/nokov_python_sdk/nokovpy-3.0.1-py3-none-any.whl}"
ENV_NAME="${NOKOV_CONDA_ENV:-nokov-ego-validation}"
ENV_FILE="${PROJECT_DIR}/environment.yml"

if [[ "$(uname -s)" != "Linux" ]]; then
  echo "error: this setup script is for Linux" >&2
  exit 2
fi
command -v conda >/dev/null 2>&1 || {
  echo "error: Conda is required; install Miniconda/Anaconda and retry" >&2
  exit 2
}
if [[ ! -f "${SDK_WHEEL}" ]]; then
  echo "error: NOKOV SDK wheel not found: ${SDK_WHEEL}" >&2
  echo "copy the vendor wheel to vendor/nokov_python_sdk/ or pass its path as argument 1" >&2
  exit 2
fi

if conda env list | awk 'NF && $1 !~ /^#/ {print $1}' | grep -Fxq "${ENV_NAME}"; then
  echo "Updating Conda environment: ${ENV_NAME}"
  conda env update --name "${ENV_NAME}" --file "${ENV_FILE}"
else
  echo "Creating Conda environment: ${ENV_NAME}"
  conda env create --name "${ENV_NAME}" --file "${ENV_FILE}"
fi

conda run --no-capture-output -n "${ENV_NAME}" \
  python -m pip install "${SDK_WHEEL}"
conda run --no-capture-output -n "${ENV_NAME}" \
  python -m pip install -r "${SCRIPT_DIR}/requirements-calibration.txt"
conda run --no-capture-output -n "${ENV_NAME}" \
  python "${SCRIPT_DIR}/check_nokov_linux_environment.py" \
  --sdk-wheel "${SDK_WHEEL}" \
  --skip-live
conda run --no-capture-output -n "${ENV_NAME}" python "${SCRIPT_DIR}/test_capture_core.py"
conda run --no-capture-output -n "${ENV_NAME}" python "${SCRIPT_DIR}/test_sync_core.py"
conda run --no-capture-output -n "${ENV_NAME}" python "${SCRIPT_DIR}/test_spatial_calibration_core.py"
conda run --no-capture-output -n "${ENV_NAME}" python "${SCRIPT_DIR}/test_camera_alignment_core.py"

echo
echo "Linux NOKOV Conda environment is ready: ${ENV_NAME}"
echo "Activate with: conda activate ${ENV_NAME}"
echo "Start XINGYING, then run: ./tools/list_nokov_assets_linux.sh"
