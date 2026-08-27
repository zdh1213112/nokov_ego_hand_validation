#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SESSION_DIR="${1:-${PACKAGE_DIR}/sessions/session_001}"
EVALUATION_DIR="${SESSION_DIR}/evaluation"

mkdir -p "${EVALUATION_DIR}"

python3 "${SCRIPT_DIR}/check_session.py" "${SESSION_DIR}" --stage capture \
  --json-out "${EVALUATION_DIR}/session_preflight.json"

if [[ -s "${SESSION_DIR}/nokov/hand24.trc" ]]; then
  NOKOV_INPUT="${SESSION_DIR}/nokov/hand24.trc"
  NOKOV_REPORT="${EVALUATION_DIR}/nokov_trc_inspection.json"
  MARKER_TEMPLATE="${EVALUATION_DIR}/marker_names_from_trc.txt"
elif [[ -s "${SESSION_DIR}/nokov/nokov_markers.csv" ]]; then
  NOKOV_INPUT="${SESSION_DIR}/nokov/nokov_markers.csv"
  NOKOV_REPORT="${EVALUATION_DIR}/nokov_sdk_csv_inspection.json"
  MARKER_TEMPLATE="${EVALUATION_DIR}/marker_names_from_sdk_csv.txt"
else
  echo "缺少 NOKOV 点数据：hand24.trc 或 nokov_markers.csv" >&2
  exit 2
fi

python3 "${SCRIPT_DIR}/inspect_nokov_hand_export.py" \
  "${NOKOV_INPUT}" \
  --expected-markers 24 \
  --json-out "${NOKOV_REPORT}" \
  --write-marker-names "${MARKER_TEMPLATE}"

if [[ -s "${SESSION_DIR}/nokov/hand24.c3d" ]]; then
  python3 "${SCRIPT_DIR}/inspect_nokov_hand_export.py" \
    "${SESSION_DIR}/nokov/hand24.c3d" \
    --expected-markers 24 \
    --json-out "${EVALUATION_DIR}/nokov_c3d_inspection.json"
fi

echo
echo "第一轮检查完成，结果位于：${EVALUATION_DIR}"
