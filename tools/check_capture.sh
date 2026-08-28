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
else
  echo "缺少 NOKOV 点数据：hand24.trc 或 nokov_markers.csv" >&2
  exit 2
fi

if [[ "${NOKOV_INPUT}" == *.csv ]]; then
  ASSET_JSON="${SESSION_DIR}/nokov/asset_descriptions.json"
  mapfile -t HAND_SETS < <(python3 - "${ASSET_JSON}" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
if path.is_file():
    data = json.loads(path.read_text(encoding="utf-8-sig"))
    for name, markers in data.get("marker_sets", {}).items():
        if len(markers) == 24:
            print(name)
PY
  )
  if [[ ${#HAND_SETS[@]} -eq 0 ]]; then
    python3 "${SCRIPT_DIR}/inspect_nokov_hand_export.py" \
      "${NOKOV_INPUT}" \
      --expected-markers 0 \
      --json-out "${EVALUATION_DIR}/nokov_sdk_csv_inspection.json"
  else
    index=0
    for marker_set in "${HAND_SETS[@]}"; do
      python3 "${SCRIPT_DIR}/inspect_nokov_hand_export.py" \
        "${NOKOV_INPUT}" \
        --markerset "${marker_set}" \
        --expected-markers 24 \
        --json-out "${EVALUATION_DIR}/nokov_sdk_hand_${index}_inspection.json"
      index=$((index + 1))
    done
  fi
else
  python3 "${SCRIPT_DIR}/inspect_nokov_hand_export.py" \
    "${NOKOV_INPUT}" \
    --expected-markers 24 \
    --json-out "${NOKOV_REPORT}" \
    --write-marker-names "${MARKER_TEMPLATE}"
fi

if [[ -s "${SESSION_DIR}/nokov/hand24.c3d" ]]; then
  python3 "${SCRIPT_DIR}/inspect_nokov_hand_export.py" \
    "${SESSION_DIR}/nokov/hand24.c3d" \
    --expected-markers 24 \
    --json-out "${EVALUATION_DIR}/nokov_c3d_inspection.json"
fi

echo
echo "第一轮检查完成，结果位于：${EVALUATION_DIR}"
