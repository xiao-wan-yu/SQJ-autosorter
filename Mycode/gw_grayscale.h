#ifndef __GW_GRAYSCALE_H
#define __GW_GRAYSCALE_H

/**
  * @brief 感为灰度传感器（串行 IO 接口）驱动
  * @note  两个灰度（GRAY1/GRAY3）都走感为官方例程的"IO 模式"（GPIO 模拟时钟），
  *        与 I2C 总线完全隔离。引脚由 CubeMX 配置（见 main.h / gpio.c）：
  *          GRAY1：DAT=PB4（输入上拉）、CLK=PB9（推挽输出）
  *          GRAY3：DAT=PB6（输入上拉）、CLK=PB7（推挽输出）
  *         CLK 下降沿采样 DAT 一位，循环 8 次读回 8 路数字量（0=深、1=浅）
  */

#include <stdint.h>

/* ==================== 灰度编号（作 GRAY_Data[] 索引，第0号废弃） ==================== */
#define GRAY1  1
#define GRAY2  2
#define GRAY3  3

extern uint8_t GRAY_Data[4][8];   // GRAY_Data[GRAYx][0~7] 对应探头1~8，串行读回的数字量：0=深（黑）、1=浅（白）
                                  // 深/浅判断由调用方（队友任务代码）自己写，例：GRAY_Data[GRAYx][i] == 0 → 深

void GRAY_Update(void);           // 一次刷新：GRAY1、GRAY3 各串行读 8 路数字量进 GRAY_Data
void GRAY1_Serial_Update(void);   // 单独刷新 GRAY1（串行接口）
void GRAY3_Serial_Update(void);   // 单独刷新 GRAY3（串行接口）

#endif
