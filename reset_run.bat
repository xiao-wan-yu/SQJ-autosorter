@echo off
REM ============================================================
REM  复位并运行（不烧录）—— 让板子上的固件立刻重头跑起来
REM  用途：
REM    1) 用别的工具（F5 调试 / CubeIDE / 别人的电脑）烧完，
REM       板子停在复位向量没反应时，跑一下这个，不用断电；
REM    2) 调试会话结束后板子被调试器按住（halt）时；
REM    3) 想不拔电重启整块板子时（等价于按复位键 + 立即运行）。
REM
REM  原理：SYSRESETREQ 系统复位 -> 内核停在复位向量 ->
REM        写 DHCSR(0xE000EDF0)=0xA05F0001 清掉 C_HALT 放行内核。
REM        （软复位不会清 C_HALT，这也是"以前必须断电"的原因）
REM ============================================================
setlocal

REM ---- 工具路径（换电脑/换 CubeIDE 版本时改这里，与 flash.bat 保持一致）----
set "IDE_ROOT=D:\STM32CubeIde\STM32CubeIDE_1.19.0\STM32CubeIDE"
set "OPENOCD=%IDE_ROOT%\plugins\com.st.stm32cube.ide.mcu.externaltools.openocd.win32_2.4.500.202604080855\tools\bin\openocd.exe"
set "OCD_SCRIPTS=%IDE_ROOT%\plugins\com.st.stm32cube.ide.mcu.debug.openocd_2.3.400.202606220929\resources\openocd\st_scripts"

set "SRC=%~dp0"
REM 去掉末尾反斜杠，否则 cmd 参数解析会把 "路径\" 当成转义
set "SRC=%SRC:~0,-1%"

echo [INFO] Reset the MCU and let it run (no flash write).

REM 说明：OpenOCD 的 -c 命令是在 init 之前执行的，
REM       而 reset/halt/mww/mdw 这些"目标命令"要 init 之后才存在，
REM       所以这里必须先 -c "init"（否则会报 invalid command name "reset"，
REM       并且后面所有 -c 命令都会被跳过 —— 2026.9.24 实测踩过）。
"%OPENOCD%" -s "%SRC%" -s "%OCD_SCRIPTS%" ^
    -f daplink_wireless.cfg ^
    -f target/stm32f4x.cfg ^
    -c "init" ^
    -c "echo {[STEP1] SYSRESETREQ : reset and halt at vector table}" ^
    -c "reset halt" ^
    -c "sleep 100" ^
    -c "echo {[STEP2] clear DHCSR.C_HALT : release the core}" ^
    -c "mww 0xE000EDF0 0xA05F0001" ^
    -c "sleep 100" ^
    -c "mww 0xE000EDF0 0xA05F0001" ^
    -c "sleep 100" ^
    -c "echo {[STEP3] DHCSR readback (S_HALT bit should be 0 = running)}" ^
    -c "mdw 0xE000EDF0 1" ^
    -c "exit" > "%TEMP%\openocd_reset.log" 2>&1

set "OPENOCD_EXIT=%errorlevel%"
type "%TEMP%\openocd_reset.log"

findstr /C:"Error:" "%TEMP%\openocd_reset.log" >nul
if not errorlevel 1 (
    echo.
    echo [WARN] OpenOCD reported an error ^(see log above^).
    echo        Check: 无线 DAP-Link 是否插好/配对成功, SWD 接线 PA13/PA14/GND/3V3,
    echo        以及 daplink_wireless.cfg 里的速度 ^(500 -> 改 100 更稳^)。
    exit /b 1
)

echo.
echo [OK] MCU reset done. Firmware should be running from the reset vector now.
exit /b 0
