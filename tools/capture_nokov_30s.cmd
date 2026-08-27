@echo off
chcp 65001 >nul
setlocal
set "SCRIPT_DIR=%~dp0"
set "ROOT_DIR=%SCRIPT_DIR%..\"
set "PYTHON=%ROOT_DIR%.venv\Scripts\python.exe"
set "OUTPUT_DIR=%SCRIPT_DIR%..\sessions\session_001\nokov"
set "SERVER=10.1.1.198"
set "HEAD_BODY=head_rigidbody"

if not exist "%PYTHON%" (
  echo [失败] 尚未安装项目环境，请先运行 setup_nokov_windows.cmd。
  pause
  exit /b 2
)

set /p "SERVER=请输入 XINGYING SDK 服务地址 [10.1.1.198]: "
if not defined SERVER set "SERVER=10.1.1.198"
set /p "HAND_SET=请输入 asset_descriptions.json 中准确的 Hand(24) MarkerSet 名称: "
if not defined HAND_SET (
  echo [失败] MarkerSet 名称不能为空。请先运行 list_nokov_assets.cmd。
  pause
  exit /b 2
)
set /p "HEAD_BODY=请输入头部刚体名称 [head_rigidbody]: "
if not defined HEAD_BODY set "HEAD_BODY=head_rigidbody"

echo 5 秒后开始采集，持续 30 秒。现在启动 EGO 录制。
"%PYTHON%" "%SCRIPT_DIR%capture_nokov_hand24.py" --server "%SERVER%" --output "%OUTPUT_DIR%" --hand-markerset "%HAND_SET%" --head-rigidbody "%HEAD_BODY%" --duration 30 --start-delay 5

echo.
echo NOKOV SDK 数据目录：%OUTPUT_DIR%
echo 注意：SDK CSV 不代替 XINGYING 的 CAP 原始工程；请同时在 XINGYING 中录制。
pause
