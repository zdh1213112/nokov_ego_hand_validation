@echo off
chcp 65001 >nul
setlocal
set "SCRIPT_DIR=%~dp0"
set "ROOT_DIR=%SCRIPT_DIR%..\"
set "PYTHON=%ROOT_DIR%.venv\Scripts\python.exe"
set "OUTPUT_DIR=%SCRIPT_DIR%..\sessions\session_001\nokov"
set "SERVER=10.1.1.198"

if not exist "%PYTHON%" (
  echo [失败] 尚未安装项目环境，请先运行 setup_nokov_windows.cmd。
  pause
  exit /b 2
)
set /p "SERVER=请输入 XINGYING SDK 服务地址 [10.1.1.198]: "
if not defined SERVER set "SERVER=10.1.1.198"

"%PYTHON%" "%SCRIPT_DIR%capture_nokov_hand24.py" --server "%SERVER%" --output "%OUTPUT_DIR%" --list-only

echo.
echo MarkerSet 和刚体列表保存在：%OUTPUT_DIR%\asset_descriptions.json
pause
