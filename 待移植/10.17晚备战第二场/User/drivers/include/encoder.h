#ifndef __ENCODER_H
#define __ENCODER_H 		

/*-------------------------------文件包含--------------------------------*/
#include "stm32f10x.h"
volatile extern int encoder1_num;
volatile extern int encoder2_num;
volatile extern int encoder3_num;
volatile extern int encoder4_num;
void Encoder_Init_TIM2(void);          //编码器定时器初始化
void Encoder_Init_TIM3(void);
void Encoder_Init_TIM4(void);
void Encoder_Init_TIM5(void);

//TIM_GetCounter(TIM2);                //读取计数器的值

#endif





























