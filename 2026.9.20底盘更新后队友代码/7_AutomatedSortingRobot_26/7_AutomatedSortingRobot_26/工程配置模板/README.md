# STM32 工程配置模板（烧录 + 代码提示）

本文件夹是从 `7_AutomatedSortingRobot_26` 工程提取出来的**可移植配置**，
用来给**新建的 STM32 工程**一键配上：
- ✅ 3 个烧录选项（无线 DAPLink / 有线 ST-Link / 有线 DAPLink）
- ✅ clangd 代码提示 + C/C++ 扩展兜底
- ✅ 一键编译（build.bat）

---

## 一、这些文件分别是干什么的

| 文件 | 作用 | 移植时要不要改 |
|---|---|---|
| `.vscode/launch.json` | F5 烧录的 3 个配置 | 要改：工程名（elf 名）；换电脑改 CubeIDE 路径 |
| `.vscode/settings.json` | clangd 参数、自动弹出提示、C/C++ 兜底 | 换电脑改 query-driver 路径 |
| `.vscode/c_cpp_properties.json` | C/C++ 扩展的头文件路径 | 换电脑改 compilerPath |
| `.vscode/tasks.json` | Ctrl+Shift+B 编译任务 | 不用改 |
| `.clangd` | clangd 编译数据库位置 | 换位置改 CompilationDatabase |
| `daplink_wireless.cfg` | 无线 DAPLink 接口（OpenOCD） | 不用改（想调速度就改 500） |
| `esp32_remote_bitbang.cfg` | ESP32 无线（备用，已不用） | 不用改 |
| `build.bat` | 一键编译脚本 | 换电脑改顶部 IDE_ROOT 等 4 行 |

---

## 二、移植步骤（新工程）

1. **复制文件**：把本文件夹里的东西按相同位置拷进新工程根目录：
   - `.vscode/` 整个文件夹 → 新工程根目录 `.vscode/`
   - `.clangd`、`daplink_wireless.cfg`、`esp32_remote_bitbang.cfg`、`build.bat` → 新工程根目录

2. **改工程名（2 处）**：
   - `launch.json` → 每个配置的 `executable` 里 `7_AutomatedSortingRobot_26.elf` 改成新工程的 elf 名
   - `CMakeLists.txt` → `set(CMAKE_PROJECT_NAME 新工程名)`
   - 注意：elf 名 = CMake 工程名，以编译后 `build/Debug/` 里实际生成的文件名为准

3. **先编译一次**（终端跑 `build.bat`，或 VS Code 里 `Ctrl+Shift+B`）：
   - 这一步会生成 `build/Debug/compile_commands.json`，clangd 和 C/C++ 扩展都靠它出提示
   - 如果新工程是 STM32CubeMX 生成的（自带 CMake），直接能编；结构不同的话，把源码文件加进 `CMakeLists.txt`

4. **用 "STM32" 配置文件打开新工程**（VS Code 右下角切换 Profile）：
   - clangd / C/C++ 扩展是在这个 Profile 里启用的，Profile 是全局的，跟着 VS Code 走，不用拷贝

5. **检查提示**：打开任意 `.c` 文件，等 clangd 后台索引几秒，打字就有候选了

---

## 三、换电脑 / 换 CubeIDE 版本时，要改的 5 处绝对路径

下面这些路径是本机（STM32CubeIDE 1.19.0 @ D 盘）的，换机器必须同步改：

1. `build.bat` 顶部：`IDE_ROOT`、`GCC_BIN`、`NINJA_BIN`、`CMAKE`（4 行）
2. `launch.json`：每个配置里的 `serverpath`（openocd）和 `gdbPath`
3. `settings.json`：`--query-driver=...`（arm-none-eabi-gcc 的目录，2 行）
4. `c_cpp_properties.json`：`compilerPath`
5. `.clangd`：`CompilationDatabase`（新工程在别的盘/路径时要改）

> 规律：只要 CubeIDE 版本和安装路径一致，第 1~4 点都不用动；只有跨电脑或跨版本才需要改。

---

## 四、备份

把整个 `工程配置模板` 文件夹上传到你的网盘 / 私有 Git 仓库，以后任何电脑、任何新工程都能快速套用。
