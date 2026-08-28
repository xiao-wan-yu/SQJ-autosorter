@echo off
REM ============================================================
REM  One-click Flash and Run - MCU auto-resets and runs new firmware
REM  Usage: double-click, or VS Code task "Flash and Run"
REM  Method: OpenOCD "program ... verify reset exit"
REM          soft-reset via SWD (SYSRESETREQ), no power cycle needed
REM  Note: OpenOCD may exit non-zero even when flashing succeeds,
REM        so success is detected from the log "Verified OK".
REM ============================================================
setlocal

REM ---- Tool paths (edit these if you move to another PC) ----
set "IDE_ROOT=D:\STM32CubeIde\STM32CubeIDE_1.19.0\STM32CubeIDE"
set "OPENOCD=%IDE_ROOT%\plugins\com.st.stm32cube.ide.mcu.externaltools.openocd.win32_2.4.500.202604080855\tools\bin\openocd.exe"
set "OCD_SCRIPTS=%IDE_ROOT%\plugins\com.st.stm32cube.ide.mcu.debug.openocd_2.3.400.202606220929\resources\openocd\st_scripts"

set "SRC=%~dp0"
REM remove trailing backslash, otherwise cmd arg parsing breaks on "path\"
set "SRC=%SRC:~0,-1%"
set "ELF=%SRC%\build\Debug\7_AutomatedSortingRobot_26.elf"
REM OpenOCD -c runs TCL; backslashes are escape chars, so use forward slashes here
set "ELF_TCL=%SRC:\=/%/build/Debug/7_AutomatedSortingRobot_26.elf"

REM OpenOCD program command requires a path without spaces
if not "%SRC%"=="%SRC: =%" (
    echo [ERROR] Project path contains spaces; unsupported. Move project to a path without spaces.
    exit /b 1
)

if not exist "%ELF%" (
    echo [ERROR] Firmware not found: %ELF%
    echo         Build first with Ctrl+Shift+B.
    exit /b 1
)

echo [INFO] Flashing %ELF%
echo [INFO] MCU will auto-reset and run after flashing. No power cycle needed.

"%OPENOCD%" -s "%SRC%" -s "%OCD_SCRIPTS%" ^
    -f daplink_wireless.cfg ^
    -f target/stm32f4x.cfg ^
    -c "program %ELF_TCL% verify reset exit" > "%TEMP%\openocd_flash.log" 2>&1

set "OPENOCD_EXIT=%errorlevel%"
type "%TEMP%\openocd_flash.log"

findstr /C:"** Verified OK **" "%TEMP%\openocd_flash.log" >nul
if errorlevel 1 goto :flashfail

echo.
echo [OK] Flash done. New firmware is running now!
exit /b 0

:flashfail
echo.
echo [ERROR] Flash failed! Check:
echo         1. Wireless DAP-Link plugged to USB and wired to SWD PA13/PA14
echo         2. Wireless DAP-Link paired successfully
echo         3. If using ST-Link wired flashing, use F5 debug instead
exit /b 1