#ifndef __SERIALPLOT_H
#define __SERIALPLOT_H

#include <stdint.h>   // 速度环调参函数用 uint8_t（本头文件被 pid.c 等引用，自带更稳）

void SERIALPLOT_ChangeParam(char *string);
void SERIALPLOT_PIDAdjustParam(void);
void SERIALPLOT_SpeedTuneLoop(uint8_t wheel);   // 单轮速度环调参（临时调试）：wheel = 轮号 CHASSIS_MOTOR_LF/LB/RB/RF
void SERIALPLOT_ChassisTestLoop(void);          // 四轮联动测试（临时调试）：串口 vy/vx 给整体速度，输出8通道(4目标+4实际)
                                                // ★2026-09-27 位置式速度环接入后仍可用（PWM 行已改打"当前生效那套"）

/* ==================== 四轮实际值实时输出（2026-09-25 加） ====================
   串口1 定时打四个轮子的**实际速度**(cm/s)，一行 4 个数字，SerialPlot 直接就是 4 条曲线：
     通道1=左前LF  通道2=左后LB  通道3=右后RB  通道4=右前RF（顺序同 chassis.h 的 ChassisMotorIndex）
   用法：主循环里按下面两个宏的要求调用 SERIALPLOT_WheelActualPump() 即可（调用点见 serialplot.c） */
#define WHEEL_ACT_SEND_EN   0  // 上电默认开关：1=上电就开始发   0=上电不发（串口1发单键 'v' 现场打开）
#define WHEEL_ACT_SEND_MS   20U   // 发送间隔(ms)：= 底盘控制周期(20ms)；嫌刷屏/嫌串口1被占就改 50 或 100
extern uint8_t serialplot_wheel_on;             // 运行期开关（1=发 / 0=停），串口1单键 'v' 切换
void SERIALPLOT_WheelActualPump(void);          // 四轮实际值：非阻塞、内部按 WHEEL_ACT_SEND_MS 节流

#endif
