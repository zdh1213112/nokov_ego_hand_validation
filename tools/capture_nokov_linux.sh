#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PYTHON_RUN="${SCRIPT_DIR}/run_nokov_python.sh"
SDK_WHEEL="${PROJECT_DIR}/vendor/nokov_python_sdk/nokovpy-3.0.1-py3-none-any.whl"

session_name=""
mode="bimanual"
server="10.1.1.198"
head_body="head_rigidbody"
left_hand="Body1_Left"
right_hand="Body1_Right"
duration="0"
start_delay="5"
queue_size="1024"

usage() {
  cat <<'EOF'
Usage:
  capture_nokov_linux.sh --session NAME [options]

Options:
  --mode rigid|bimanual       default: bimanual
  --server ADDRESS            default: 10.1.1.198
  --head-rigidbody NAME       default: head_rigidbody
  --left-hand NAME            default: Body1_Left
  --right-hand NAME           default: Body1_Right
  --duration SECONDS          0 means until Ctrl+C; default: 0
  --start-delay SECONDS       default: 5
  --queue-size N              default: 1024
  -h, --help

Run list_nokov_assets_linux.sh first and use the exact live asset names.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --session) session_name="${2:-}"; shift 2 ;;
    --mode) mode="${2:-}"; shift 2 ;;
    --server) server="${2:-}"; shift 2 ;;
    --head-rigidbody) head_body="${2:-}"; shift 2 ;;
    --left-hand) left_hand="${2:-}"; shift 2 ;;
    --right-hand) right_hand="${2:-}"; shift 2 ;;
    --duration) duration="${2:-}"; shift 2 ;;
    --start-delay) start_delay="${2:-}"; shift 2 ;;
    --queue-size) queue_size="${2:-}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "error: unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "${session_name}" ]]; then
  echo "error: --session is required" >&2
  usage >&2
  exit 2
fi
if [[ ! "${session_name}" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "error: session name may contain only letters, digits, dot, underscore and hyphen" >&2
  exit 2
fi
if [[ "${mode}" != "rigid" && "${mode}" != "bimanual" ]]; then
  echo "error: --mode must be rigid or bimanual" >&2
  exit 2
fi
if [[ ! -x "${PYTHON_RUN}" ]]; then
  echo "error: Python runner is missing: ${PYTHON_RUN}" >&2
  exit 2
fi

session_dir="${PROJECT_DIR}/sessions/${session_name}"
mkdir -p \
  "${session_dir}/ego" \
  "${session_dir}/nokov/raw_capture" \
  "${session_dir}/synchronization" \
  "${session_dir}/calibration" \
  "${session_dir}/config" \
  "${session_dir}/evaluation"

command=(
  "${PYTHON_RUN}" "${SCRIPT_DIR}/capture_nokov_hand24.py"
  --server "${server}"
  --output "${session_dir}/nokov"
  --head-rigidbody "${head_body}"
  --duration "${duration}"
  --start-delay "${start_delay}"
  --queue-size "${queue_size}"
)
if [[ -f "${SDK_WHEEL}" ]]; then
  command+=(--sdk-wheel "${SDK_WHEEL}")
fi
if [[ "${mode}" == "rigid" ]]; then
  command+=(--rigid-only)
else
  command+=(
    --hand-markerset "${left_hand}"
    --hand-markerset "${right_hand}"
    --expected-hand-markers 24
  )
fi

echo "Session: ${session_dir}"
echo "Mode: ${mode}"
echo "Start XINGYING CAP recording separately before the countdown finishes."
printf 'Command:'
printf ' %q' "${command[@]}"
printf '\n'
"${command[@]}"

echo
echo "NOKOV SDK capture complete: ${session_dir}/nokov"
echo "Copy the matching EGO MCAP to: ${session_dir}/ego/"
echo "Archive the XINGYING CAP under: ${session_dir}/nokov/raw_capture/"
