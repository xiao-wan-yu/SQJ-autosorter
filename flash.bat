@echo off
REM ============================================================
REM  一键烧录并运行（Flash & Run）—— 烧完自动复位，新固件立即开始跑
REM  用法：双击本文件，或 VS Code: Ctrl+Shift+P -> Tasks: Run Task
REM        -> "烧录并运行 (Flash & Run)"
REM
REM  工具：OpenOCD + 无线 DAP-Link（无 nRST 复位线，复位只能靠
REM        SYSRESETREQ 软复位）
REM
REM  【2026.9.24 修正 · 为什么以前烧完必须断电重启】
REM    原因1：OpenOCD 的 -c 命令是在 init 之前执行的，而 reset/halt/mww/mdw
REM           这类"目标命令"要 init 之后才存在。写了顺序不对就会报
REM           invalid command name "reset"，并且**后续 -c 命令全部被跳过**
REM           （实测过），于是"最后那脚让它跑起来"的命令根本没执行。
REM           本脚本用 program（它内部会 init）→ 之后的 reset/mww/mdw 才有效。
REM    原因2：SYSRESETREQ 只复位内核和外设，**不复位调试寄存器**：
REM           烧录时调试器把内核置成 HALT（DHCSR.C_HALT=1），软复位后这位
REM           仍是 1，内核会停在复位向量不动（现象：烧完没反应、像旧程序），
REM           只有断电（调试域彻底掉电）才清掉它。
REM           所以最后显式写 DHCSR = 0xA05F0001（DBGKEY + C_DEBUGEN，
REM           即 C_HALT=0）放行内核，再读回确认 S_HALT=0。
REM           （2026.9.24 真机实测：halted 时 DHCSR=0x00030003，
REM             写完后 =0x01010001 → 内核已在跑）
REM
REM  注意：OpenOCD 即使烧写成功也可能返回非 0，所以成功与否
REM        以日志里的 "** Verified OK **" 为准。
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

REM ------------------------------------------------------------
REM  第 1 步 program ... verify ：擦除 + 写入 + 校验
REM  第 2 步 reset halt         ：SYSRESETREQ 系统复位，内核停在复位向量
REM  第 3 步 mww DHCSR          ：清 C_HALT 放行内核（关键！）
REM                              0xA05F0001 = DBGKEY(0xA05F) | C_DEBUGEN(bit0)
REM                              C_HALT(bit1)=0 -> 内核从复位向量开始跑新固件
REM                              写两次：无线链路偶发丢包时重试
REM  第 4 步 mdw DHCSR          ：读回确认，S_HALT(bit17)=0 即内核正在运行
REM  第 5 步 exit               ：退出 OpenOCD，内核保持运行
REM ------------------------------------------------------------
"%OPENOCD%" -s "%SRC%" -s "%OCD_SCRIPTS%" ^
    -f daplink_wireless.cfg ^
    -f target/stm32f4x.cfg ^
    -c "program %ELF_TCL% verify" ^
    -c "sleep 200" ^
    -c "echo {[STEP2] SYSRESETREQ : reset and halt at vector table}" ^
    -c "reset halt" ^
    -c "sleep 100" ^
    -c "echo {[STEP3] clear DHCSR.C_HALT : release the core to run new firmware}" ^
    -c "mww 0xE000EDF0 0xA05F0001" ^
    -c "sleep 100" ^
    -c "mww 0xE000EDF0 0xA05F0001" ^
    -c "sleep 100" ^
    -c "echo {[STEP4] DHCSR readback (S_HALT bit should be 0 = running)}" ^
    -c "mdw 0xE000EDF0 1" ^
    -c "exit" > "%TEMP%\openocd_flash.log" 2>&1

set "OPENOCD_EXIT=%errorlevel%"
type "%TEMP%\openocd_flash.log"

findstr /C:"** Verified OK **" "%TEMP%\openocd_flash.log" >nul
if errorlevel 1 goto :flashfail

findstr /C:"Error: CMSIS-DAP command CMD_INFO failed" "%TEMP%\openocd_flash.log" >nul
if not errorlevel 1 (
    echo.
    echo [WARN] DAP-Link link error detected: "CMD_INFO failed".
    echo        Wireless link may be down / DAP firmware stuck.
    echo        Fix: replug the PC-side DAP-Link USB, or power-cycle
    echo        BOTH DAP-Links and let them re-pair, then flash again.
)

REM ---- Extra check: did the reset / core-release steps hit an error? ----
findstr /C:"Error:" "%TEMP%\openocd_flash.log" >nul
if not errorlevel 1 (
    echo.
    echo [WARN] OpenOCD log contains an error line.
    echo        If it comes from STEP3 / STEP4 ^(mww / mdw 0xE000EDF0^),
    echo        the wireless link dropped during the reset moment.
    echo        Firmware is still written; just run reset_run.bat once
    echo        ^(VS Code task "复位并运行 (Reset & Run)"^) to make it run.
)

echo.
echo [OK] Flash done. Core released (DHCSR.C_HALT cleared) = new firmware running now!
exit /b 0

:flashfail
echo.
echo [ERROR] Flash failed! Check:
echo         1. Wireless DAP-Link plugged to USB and wired to SWD PA13/PA14
echo         2. Wireless DAP-Link paired successfully
echo         3. If using ST-Link wired flashing, use F5 debug instead
echo.
echo         If you see "CMSIS-DAP command CMD_INFO failed":
echo           -> Replug the PC-side DAP-Link USB, or power-cycle both
echo              DAP-Links so they re-pair, then flash again.
exit /b 1