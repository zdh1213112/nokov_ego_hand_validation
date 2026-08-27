#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SESSION_DIR="${1:-${PACKAGE_DIR}/sessions/session_001}"
EVALUATION_DIR="${SESSION_DIR}/evaluation"

mkdir -p "${EVALUATION_DIR}"

python3 "${SCRIPT_DIR}/check_session.py" "${SESSION_DIR}" --stage calibrated \
  --json-out "${EVALUATION_DIR}/calibrated_session_preflight.json"

echo
echo "标定阶段检查通过。可以开始24→21转换和误差评价。"
