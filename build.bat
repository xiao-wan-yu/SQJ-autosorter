@echo off
REM ============================================================
REM  STM32 one-click build script
REM  使用 CMakePresets.json + PATH 中的 cmake / arm-none-eabi-gcc
REM  （与队友工程 8_ElectronicDesignContest_26 一致，无需硬编码 CubeIDE 路径）
REM ============================================================
setlocal
REM Fallback: cmake not on PATH -> prepend STM32 Cube bundle dirs
where cmake >nul 2>nul
if errorlevel 1 (
    for /d %%D in ("%LOCALAPPDATA%\stm32cube\bundles\cmake\*") do set "PATH=%%D\bin;%PATH%"
)

cd /d "%~dp0"

REM Configure first if not configured yet
if not exist "build\Debug\build.ninja" (
    echo [INFO] First configure...
    cmake --preset Debug
    if errorlevel 1 exit /b 1
)

echo [INFO] Building...
cmake --build --preset Debug -- -j4
exit /b %errorlevel%
