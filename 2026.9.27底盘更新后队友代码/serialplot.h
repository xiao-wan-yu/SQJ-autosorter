#ifndef __SERIALPLOT_H
#define __SERIALPLOT_H

#include <stdint.h>   // 速度环调参函数用 uint8_t（本头文件被 pid.c 等引用，自带更稳）

void SERIALPLOT_ChangeParam(char *string);
void SERIALPLOT_PIDAdjustParam(void);
void SERIALPLOT_SpeedTuneLoop(uint8_t wheel);   // 单轮速度环调参（临时调试）：wheel = 轮号 CHASSIS_MOTOR_LF/LB/RB/RF
void SERIALPLOT_ChassisTestLoop(void);          // 四轮联动测试（临时调试）：串口 vy/vx 给整体速度，输出8通道(4目标+4实际)
void SERIALPLOT_SpeedPidDebug(void);            // 速度环PID内部量调试（临时调试）：往复横移，输出6通道(目标/实测/输出/P项/I项/D项)

#endif
