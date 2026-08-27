#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

REQUIRED_FILES=(
  "vendor/xingying/XINGYING_4.6.0.7923_Windows_x64.exe"
  "vendor/nokov_python_sdk/xing_python_sdk_4.1.0.5645/dist/nokovpy-3.0.1-py3-none-any.whl"
  "assets/nokov_calibration/CalWand.cap"
  "assets/nokov_calibration/CalFrame.cap"
  "ego_wilor/third_party/orbbec_sdk/lib/libOrbbecSDK.so.2.9.0"
  "ego_wilor/models/wilor/wilor_final.ckpt"
  "ego_wilor/models/wilor/detector.pt"
  "ego_wilor/models/mano/MANO_RIGHT.pkl"
  "ego_wilor/models/mano/MANO_LEFT.pkl"
  "reference_data/ego_multiview/fusion/accepted.jsonl"
  "reference_data/ego_multiview/normalized_multiview/multiview_frames.csv"
)

MISSING_ASSETS=0
for relative_path in "${REQUIRED_FILES[@]}"; do
  if [[ ! -s "${PACKAGE_DIR}/${relative_path}" ]]; then
    echo "[MISSING] ${relative_path}" >&2
    MISSING_ASSETS=$((MISSING_ASSETS + 1))
  fi
done
if ((MISSING_ASSETS > 0)); then
  echo "项目缺少 ${MISSING_ASSETS} 个必需资产。" >&2
  exit 2
fi

BROKEN_LINK="$(find -L "${PACKAGE_DIR}" -type l -print -quit)"
if [[ -n "${BROKEN_LINK}" ]]; then
  echo "[MISSING] 失效软链接：${BROKEN_LINK}" >&2
  exit 2
fi

echo "Required local assets: OK"

python3 "${SCRIPT_DIR}/inspect_ego_output.py" \
  --fusion "${PACKAGE_DIR}/reference_data/ego_multiview/fusion/accepted.jsonl" \
  --timestamps "${PACKAGE_DIR}/reference_data/ego_multiview/normalized_multiview/multiview_frames.csv"

python3 "${SCRIPT_DIR}/check_session.py" \
  "${PACKAGE_DIR}/sessions/session_001" \
  --allow-incomplete

echo
echo "项目结构正常。session_001 显示 MISSING 是正常的：需要现场放入 TRC/C3D/SDK CSV、MCAP 和真实标定。"
