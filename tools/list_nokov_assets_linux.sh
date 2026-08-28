#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PYTHON_RUN="${SCRIPT_DIR}/run_nokov_python.sh"
SERVER="${1:-10.1.1.198}"
OUTPUT="${2:-${PROJECT_DIR}/sessions/_discovery/nokov}"
SDK_WHEEL="${PROJECT_DIR}/vendor/nokov_python_sdk/nokovpy-3.0.1-py3-none-any.whl"

if [[ ! -x "${PYTHON_RUN}" ]]; then
  echo "error: Python runner is missing: ${PYTHON_RUN}" >&2
  exit 2
fi

command=(
  "${PYTHON_RUN}" "${SCRIPT_DIR}/capture_nokov_hand24.py"
  --server "${SERVER}"
  --output "${OUTPUT}"
  --list-only
)
if [[ -f "${SDK_WHEEL}" ]]; then
  command+=(--sdk-wheel "${SDK_WHEEL}")
fi
"${command[@]}"

echo
echo "Live asset description: ${OUTPUT}/asset_descriptions.json"
