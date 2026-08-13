@echo off
REM ============================================================
REM  无线DAP-Link 一键烧录脚本（只烧录，不进调试，速度快）
REM  用法：VS Code 里 Ctrl+Shift+P -> "任务: 运行任务" -> "Flash 无线DAPLink"
REM  烧录完需断电重启开发板（reset_config none，不接复位线）
REM ============================================================
setlocal
set "IDE_ROOT=D:\STM32CubeIde\STM32CubeIDE_1.19.0\STM32CubeIDE"
set "GCC_BIN=%IDE_ROOT%\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin"
set "NINJA_BIN=%IDE_ROOT%\plugins\com.st.stm32cube.ide.mcu.externaltools.ninja.win32_1.1.200.202606260906\tools\bin"
set "CMAKE=%IDE_ROOT%\plugins\com.st.stm32cube.ide.mcu.externaltools.cmake.win32_1.1.200.202605190741\tools\bin\cmake.exe"
set "OPENOCD=%IDE_ROOT%\plugins\com.st.stm32cube.ide.mcu.externaltools.openocd.win32_2.4.500.202604080855\tools\bin\openocd.exe"
set "ST_SCRIPTS=%IDE_ROOT%\plugins\com.st.stm32cube.ide.mcu.debug.openocd_2.3.400.202606220929\resources\openocd\st_scripts"
set "PATH=%GCC_BIN%;%NINJA_BIN%;%PATH%"

set "SRC=%~dp0"
set "BUILD=%SRC%build\Debug"

REM 1. 编译
if not exist "%BUILD%\build.ninja" (
    echo [INFO] First configure...
    "%CMAKE%" -S "%SRC%." -B "%BUILD%" -G Ninja -DCMAKE_BUILD_TYPE=Debug
    if errorlevel 1 exit /b 1
)
echo [INFO] Building...
"%CMAKE%" --build "%BUILD%" -- -j4
if errorlevel 1 exit /b 1

REM 2. 烧录（program 一次性 擦除+写入+校验+复位，不经 gdb，比 F5 调试快很多）
echo [INFO] Flashing via wireless DAP-Link...
"%OPENOCD%" -s "%SRC%." -s "%ST_SCRIPTS%" -f daplink_wireless.cfg -f target/stm32f4x.cfg -c "program %BUILD%\7_AutomatedSortingRobot_26.elf verify reset exit"
exit /b %errorlevel%
