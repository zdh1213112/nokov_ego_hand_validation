@echo off
chcp 65001 >nul
setlocal
set "SCRIPT_DIR=%~dp0"
set "ROOT_DIR=%SCRIPT_DIR%..\"
set "VENV_DIR=%ROOT_DIR%.venv"
set "SDK_WHEEL=%ROOT_DIR%vendor\nokov_python_sdk\nokovpy-3.0.1-py3-none-any.whl"

where py >nul 2>nul
if errorlevel 1 (
  echo [失败] 没有找到 Python Launcher。请先安装 64 位 Python 3.10 或 3.11。
  pause
  exit /b 2
)

if not exist "%SDK_WHEEL%" (
  echo [失败] 没有找到 NOKOV Python SDK wheel：
  echo %SDK_WHEEL%
  echo.
  echo 请从厂家 SDK 包复制 nokovpy-3.0.1-py3-none-any.whl 到上述位置。
  pause
  exit /b 2
)

echo 创建 Python 虚拟环境：%VENV_DIR%
py -3 -m venv "%VENV_DIR%"
if errorlevel 1 goto :failed

set "PYTHON=%VENV_DIR%\Scripts\python.exe"
"%PYTHON%" -m pip install --upgrade pip
if errorlevel 1 goto :failed
"%PYTHON%" -m pip install "%SDK_WHEEL%"
if errorlevel 1 goto :failed
"%PYTHON%" -m pip install -r "%SCRIPT_DIR%requirements-sync.txt"
if errorlevel 1 goto :failed
"%PYTHON%" "%SCRIPT_DIR%check_nokov_windows_environment.py"
if errorlevel 1 goto :failed

echo.
echo [完成] 环境安装成功。下一步运行 list_nokov_assets.cmd。
pause
exit /b 0

:failed
echo.
echo [失败] 环境安装或检查失败，请查看上方错误。
pause
exit /b 2
