#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PYTHON_RUN="${SCRIPT_DIR}/run_nokov_python.sh"

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "usage: $0 SESSION_NAME [RIGID_BODY_NAME]" >&2
  exit 2
fi

SESSION_NAME="$1"
RIGID_BODY="${2:-head_rigidbody}"
SESSION_DIR="${PROJECT_DIR}/sessions/${SESSION_NAME}"
EGO_DIR="${SESSION_DIR}/ego"
NOKOV_CSV="${SESSION_DIR}/nokov/nokov_rigid_bodies.csv"
OUTPUT_DIR="${SESSION_DIR}/synchronization"

[[ -x "${PYTHON_RUN}" ]] || {
  echo "error: missing ${PYTHON_RUN}" >&2
  exit 2
}
[[ -d "${EGO_DIR}" ]] || { echo "error: missing ${EGO_DIR}" >&2; exit 2; }
[[ -f "${NOKOV_CSV}" ]] || { echo "error: missing ${NOKOV_CSV}" >&2; exit 2; }

if [[ -f "${EGO_DIR}/recording.mcap" ]]; then
  EGO_MCAP="${EGO_DIR}/recording.mcap"
else
  mapfile -d '' MCAP_FILES < <(find "${EGO_DIR}" -maxdepth 1 -type f -name '*.mcap' -print0)
  if [[ ${#MCAP_FILES[@]} -ne 1 ]]; then
    echo "error: ego/ must contain recording.mcap or exactly one .mcap file" >&2
    exit 2
  fi
  EGO_MCAP="${MCAP_FILES[0]}"
fi

"${PYTHON_RUN}" "${SCRIPT_DIR}/synchronize_ego_imu_nokov.py" \
  --ego-mcap "${EGO_MCAP}" \
  --nokov-csv "${NOKOV_CSV}" \
  --rigid-body "${RIGID_BODY}" \
  --output-dir "${OUTPUT_DIR}" \
  --nokov-time-field device_timestamp_raw \
  --nokov-time-scale 0.001 \
  --max-offset-s 30

echo "Synchronization results: ${OUTPUT_DIR}"
