#!/usr/bin/env bash
set -euo pipefail

ENV_NAME="${NOKOV_CONDA_ENV:-nokov-ego-validation}"

if [[ -n "${NOKOV_PYTHON:-}" ]]; then
  if [[ ! -x "${NOKOV_PYTHON}" ]]; then
    echo "error: NOKOV_PYTHON is not executable: ${NOKOV_PYTHON}" >&2
    exit 2
  fi
  exec "${NOKOV_PYTHON}" "$@"
fi

if [[ "${CONDA_DEFAULT_ENV:-}" == "${ENV_NAME}" ]]; then
  exec python "$@"
fi

if command -v conda >/dev/null 2>&1 \
  && conda run -n "${ENV_NAME}" python -c 'pass' >/dev/null 2>&1; then
  exec conda run --no-capture-output -n "${ENV_NAME}" python "$@"
fi

if command -v python3 >/dev/null 2>&1; then
  echo "warning: Conda environment ${ENV_NAME} was not found; using system python3" >&2
  echo "warning: this fallback is sufficient for SDK capture only when nokovpy is installed or --sdk-wheel is available" >&2
  exec python3 "$@"
fi

echo "error: no Python runtime found; run ./tools/setup_nokov_linux.sh" >&2
exit 2
