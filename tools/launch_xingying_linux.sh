#!/usr/bin/env bash
set -euo pipefail

XINGYING_BIN="${XINGYING_BIN:-/usr/local/XINGYING/bin/XINGYING}"

if [[ ! -x "${XINGYING_BIN}" ]]; then
  echo "error: XINGYING executable not found or not executable: ${XINGYING_BIN}" >&2
  exit 2
fi
if pgrep -f "^${XINGYING_BIN}([[:space:]]|$)" >/dev/null 2>&1; then
  echo "error: XINGYING is already running; do not start a second SDK server" >&2
  exit 2
fi
if [[ -z "${XDG_RUNTIME_DIR:-}" ]]; then
  candidate="/run/user/$(id -u)"
  if [[ -d "${candidate}" && -w "${candidate}" ]]; then
    export XDG_RUNTIME_DIR="${candidate}"
  else
    echo "warning: XDG_RUNTIME_DIR is unavailable; Qt may fall back to /tmp" >&2
  fi
fi
if [[ "$(id -u)" -eq 0 ]]; then
  echo "warning: XINGYING is being launched as root; prefer the logged-in desktop user" >&2
fi

echo "Starting XINGYING: ${XINGYING_BIN}"
echo "XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-<unset>}"
exec "${XINGYING_BIN}" "$@"
