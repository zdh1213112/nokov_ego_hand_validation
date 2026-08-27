@echo off
chcp 65001 >nul
setlocal
set "SCRIPT_DIR=%~dp0"
set "ROOT_DIR=%SCRIPT_DIR%..\"
set "PYTHON=%ROOT_DIR%.venv\Scripts\python.exe"
set "SERVER=10.1.1.198"
set "SESSION_NAME=session_head_sync_001"
set "HEAD_BODY=head_rigidbody"

if not exist "%PYTHON%" (
  echo [失败] 尚未安装项目环境，请先运行 setup_nokov_windows.cmd。
  pause
  exit /b 2
)

set /p "SERVER=请输入 XINGYING SDK 服务地址 [10.1.1.198]: "
if not defined SERVER set "SERVER=10.1.1.198"
set /p "SESSION_NAME=请输入 Session 名称 [session_head_sync_001]: "
if not defined SESSION_NAME set "SESSION_NAME=session_head_sync_001"
set /p "HEAD_BODY=请输入刚体名称 [head_rigidbody]: "
if not defined HEAD_BODY set "HEAD_BODY=head_rigidbody"
set "OUTPUT_DIR=%ROOT_DIR%sessions\%SESSION_NAME%\nokov"
if not exist "%ROOT_DIR%sessions\%SESSION_NAME%\ego" mkdir "%ROOT_DIR%sessions\%SESSION_NAME%\ego"
if not exist "%ROOT_DIR%sessions\%SESSION_NAME%\synchronization" mkdir "%ROOT_DIR%sessions\%SESSION_NAME%\synchronization"

echo.
echo 请确认 XINGYING 已经：加载刚体、处于播放状态、启用 SDK，并单独录制 CAP。
echo SDK CSV 输出目录：%OUTPUT_DIR%
echo EGO MCAP 请复制为：%ROOT_DIR%sessions\%SESSION_NAME%\ego\recording.mcap
echo 按 Ctrl+C 停止采集。
echo.
"%PYTHON%" "%SCRIPT_DIR%capture_nokov_hand24.py" ^
  --server "%SERVER%" ^
  --output "%OUTPUT_DIR%" ^
  --rigid-only ^
  --head-rigidbody "%HEAD_BODY%" ^
  --duration 0 ^
  --start-delay 5 ^
  --queue-size 1024

echo.
echo 采集程序已结束。
pause
