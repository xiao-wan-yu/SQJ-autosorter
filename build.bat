@echo off
REM ============================================================
REM  STM32 one-click build script (uses local STM32CubeIDE tools)
REM  Update the three paths below when moving to another PC.
REM ============================================================
setlocal
set "IDE_ROOT=D:\STM32CubeIde\STM32CubeIDE_1.19.0\STM32CubeIDE"
set "GCC_BIN=%IDE_ROOT%\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin"
set "NINJA_BIN=%IDE_ROOT%\plugins\com.st.stm32cube.ide.mcu.externaltools.ninja.win32_1.1.200.202606260906\tools\bin"
set "CMAKE=%IDE_ROOT%\plugins\com.st.stm32cube.ide.mcu.externaltools.cmake.win32_1.1.200.202605190741\tools\bin\cmake.exe"
set "PATH=%GCC_BIN%;%NINJA_BIN%;%PATH%"

set "SRC=%~dp0"
set "BUILD=%SRC%build\Debug"

REM Configure first if not configured yet
if not exist "%BUILD%\build.ninja" (
    echo [INFO] First configure...
    "%CMAKE%" -S "%SRC%." -B "%BUILD%" -G Ninja -DCMAKE_BUILD_TYPE=Debug
    if errorlevel 1 exit /b 1
)

echo [INFO] Building...
"%CMAKE%" --build "%BUILD%" -- -j4
exit /b %errorlevel%
