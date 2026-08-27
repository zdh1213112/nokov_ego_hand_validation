@echo off
chcp 65001 >nul
setlocal
set "SCRIPT_DIR=%~dp0"
set "ROOT_DIR=%SCRIPT_DIR%..\"
set "PYTHON=%ROOT_DIR%.venv\Scripts\python.exe"
set "SESSION_NAME=session_head_sync_001"
set "HEAD_BODY=head_rigidbody"

if not exist "%PYTHON%" (
  echo [失败] 尚未安装项目环境，请先运行 setup_nokov_windows.cmd。
  pause
  exit /b 2
)

set /p "SESSION_NAME=请输入 Session 名称 [session_head_sync_001]: "
if not defined SESSION_NAME set "SESSION_NAME=session_head_sync_001"
set /p "HEAD_BODY=请输入刚体名称 [head_rigidbody]: "
if not defined HEAD_BODY set "HEAD_BODY=head_rigidbody"

set "SESSION_DIR=%ROOT_DIR%sessions\%SESSION_NAME%"
set "EGO_MCAP=%SESSION_DIR%\ego\recording.mcap"
set "NOKOV_CSV=%SESSION_DIR%\nokov\nokov_rigid_bodies.csv"
set "SYNC_DIR=%SESSION_DIR%\synchronization"

if not exist "%EGO_MCAP%" (
  echo [失败] 缺少 EGO MCAP：%EGO_MCAP%
  pause
  exit /b 2
)
if not exist "%NOKOV_CSV%" (
  echo [失败] 缺少 NOKOV 刚体 CSV：%NOKOV_CSV%
  pause
  exit /b 2
)

"%PYTHON%" "%SCRIPT_DIR%synchronize_ego_imu_nokov.py" ^
  --ego-mcap "%EGO_MCAP%" ^
  --nokov-csv "%NOKOV_CSV%" ^
  --rigid-body "%HEAD_BODY%" ^
  --output-dir "%SYNC_DIR%" ^
  --max-offset-s 30

echo.
echo 同步结果目录：%SYNC_DIR%
pause
