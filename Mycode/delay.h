/**
  ******************************************************************************
  * @file    delay.h
  * @brief   DWT高精度延时库（自适应主频 + 自动检测）
  * @note    支持STM32F4、F1等系列，自动适应168MHz/84MHz等不同主频
  *          自动检测系统时钟变化并重新初始化
  ******************************************************************************
  */

#ifndef __DELAY_H
#define __DELAY_H

#ifdef __cplusplus
extern "C" {
#endif

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx.h"

/* 宏定义 --------------------------------------------------------------------*/
// 自动计算时钟相关参数（编译时优化）
#define DELAY_US_TICKS      (SystemCoreClock / 1000000UL)   // 每微秒的时钟周期数
#define DELAY_OVERFLOW_US   (0xFFFFFFFFUL / DELAY_US_TICKS) // 计数器溢出时间(us)
#define DELAY_MAX_SINGLE_US 1000000UL                       // 单次最大延时1秒

/* 函数声明 ------------------------------------------------------------------*/
void DWT_Init(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* __DELAY_H */
