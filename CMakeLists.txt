cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


#为解决支持%f问题添加--开始
# 启用 printf 浮点格式支持
add_compile_options(
    -u _printf_float                    # 关键：启用printf浮点
    -Wl,--wrap,printf                   # 可选：包装printf函数
)
# 硬件浮点设置（STM32F407 使用 Cortex-M4 FPU）
add_compile_options(
    -mcpu=cortex-m4                    # CPU 型号（必须，否则默认ARM模式编译会报 cpsid 错误）
    -mthumb                            # Thumb 指令集（Cortex-M 必须）
    -mfloat-abi=hard                    # 硬件浮点
    -mfpu=fpv4-sp-d16                   # F4系列的FPU
)
# 链接选项也要添加
add_link_options(
    -u _printf_float
    -mcpu=cortex-m4
    -mthumb
    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16
    # 注意：--specs=nano.specs 和 -T 链接脚本已由 cmake/gcc-arm-none-eabi.cmake 工具链文件添加，
    # 这里不能再加，否则 GCC 14.x 分别报 "attempt to rename spec 'link'..."
    # 和 "linker script file appears multiple times" 链接错误
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)
#为解决支持%f问题添加--结束


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()




#是否启用编译器优化，注释则不启用优化，参数为-Os则优化大小-代码最小，参数为-O2则优化性能，速度最快
#add_compile_options(-Os)



# Set the project name
set(CMAKE_PROJECT_NAME 7_AutomatedSortingRobot_26)


# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# 确保可执行文件带 .elf 后缀（工具链文件的设置未生效，这里强制）
set(CMAKE_EXECUTABLE_SUFFIX_C ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_ASM ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX ".elf")

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
    ${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CMAKE_SOURCE_DIR}/Mycode/led.c
    ${CMAKE_SOURCE_DIR}/Mycode/uart.c
    ${CMAKE_SOURCE_DIR}/Mycode/delay.c
    ${CMAKE_SOURCE_DIR}/Mycode/key.c
    ${CMAKE_SOURCE_DIR}/Mycode/mpu6050.c
    ${CMAKE_SOURCE_DIR}/Mycode/gray.c
    ${CMAKE_SOURCE_DIR}/Mycode/buzzer.c
    ${CMAKE_SOURCE_DIR}/Mycode/laser.c
    ${CMAKE_SOURCE_DIR}/Mycode/infrared.c
    ${CMAKE_SOURCE_DIR}/Mycode/gy53.c
    ${CMAKE_SOURCE_DIR}/Mycode/ultrasonic.c
    ${CMAKE_SOURCE_DIR}/Mycode/servo.c
    ${CMAKE_SOURCE_DIR}/Mycode/tb6612.c
    ${CMAKE_SOURCE_DIR}/Mycode/encoder.c
    ${CMAKE_SOURCE_DIR}/Mycode/emm_v5.c
    ${CMAKE_SOURCE_DIR}/Mycode/serialplot.c
    ${CMAKE_SOURCE_DIR}/Mycode/pid.c
    ${CMAKE_SOURCE_DIR}/Mycode/icm42688.c
    ${CMAKE_SOURCE_DIR}/Mycode/myflash.c
    ${CMAKE_SOURCE_DIR}/Mycode/storage.c
    ${CMAKE_SOURCE_DIR}/Mycode/nmos.c

    ${CMAKE_SOURCE_DIR}/Mycode/mpu6050/mpu.c
    ${CMAKE_SOURCE_DIR}/Mycode/mpu6050/inv_mpu.c
    ${CMAKE_SOURCE_DIR}/Mycode/mpu6050/iic.c
    ${CMAKE_SOURCE_DIR}/Mycode/mpu6050/inv_mpu_dmp_motion_driver.c
    ${CMAKE_SOURCE_DIR}/Mycode/icm42688/icm42688.c
    ${CMAKE_SOURCE_DIR}/Mycode/icm42688/quaternion.c
    # ${CMAKE_SOURCE_DIR}/Mycode/icm42688/ak09911c.c
    # ${CMAKE_SOURCE_DIR}/Mycode/icm42688/ak09911c_myiic.c

    
    #根据选择的oled文件做相应修改
    # ${CMAKE_SOURCE_DIR}/Mycode/oled_ui/oled_driver.c
    # ${CMAKE_SOURCE_DIR}/Mycode/oled_ui/oled_fonts.c
    # ${CMAKE_SOURCE_DIR}/Mycode/oled_ui/oled_ui_driver.c
    # ${CMAKE_SOURCE_DIR}/Mycode/oled_ui/oled_ui_menudata.c
    # ${CMAKE_SOURCE_DIR}/Mycode/oled_ui/oled_ui.c
    # ${CMAKE_SOURCE_DIR}/Mycode/oled_ui/oled.c
    ${CMAKE_SOURCE_DIR}/Mycode/oled_jiangkeda/oled.c
    ${CMAKE_SOURCE_DIR}/Mycode/oled_jiangkeda/oled_data.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_SOURCE_DIR}/Mycode
    ${CMAKE_SOURCE_DIR}/Mycode/mpu6050
    ${CMAKE_SOURCE_DIR}/Mycode/icm42688
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Include/dsp

    #根据选择的oled文件做相应修改
    # ${CMAKE_SOURCE_DIR}/Mycode/oled_ui
    ${CMAKE_SOURCE_DIR}/Mycode/oled_jiangkeda
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
    libarm_cortexM4lf_math.a
    m
)
