#ifndef __TB6612_H
#define __TB6612_H

#include "stm32f4xx_hal.h"

//====================================================================
// TB6612 四路电机驱动（新主控板 + 拓展板）
//
//   A电机: PWMA = TIM1_CH1(PE9)    AIN1=PA3   AIN2=PA4
//   B电机: PWMB = TIM1_CH2(PE11)   BIN1=PC4   BIN2=PC5
//   C电机: PWMC = TIM1_CH3(PE13)   CIN1=PB0   CIN2=PB1
//   D电机: PWMD = TIM1_CH4(PE14)   DIN1=PB2   DIN2=PE7
//   STBY = PE8（1-退出待机，0-待机）
//====================================================================

//直流减速电机号码
#define MOTOR_A    0   // 电机A
#define MOTOR_B    1   // 电机B
#define MOTOR_C    2   // 电机C
#define MOTOR_D    3   // 电机D

/* 旧板兼容别名（老测试代码可能仍引用，避免编译报错） */
#define MOTOR_Left  MOTOR_A
#define MOTOR_Right MOTOR_B

void TB6612_Init(void);                                     // STBY使能 + 启动4路PWM
void TB6612_Control(uint8_t MOTOR_Num, int8_t Motor_Speed); // 速度-100~100（百分比），正负控制转向

#endif
