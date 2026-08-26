#ifndef __PWM_H
#define __PWM_H 		

/*-------------------------------文件包含--------------------------------*/
#include "stm32f10x.h"
#define Moter1_STOP  	          TIM_SetCompare1(TIM8,0)
#define Moter1_DISABLE          TIM_SetCompare1(TIM8,0)
#define Moter1_FORWARD(duty)    GPIO_SetBits(GPIOD, GPIO_Pin_3);TIM_SetCompare1(TIM8,duty)
#define Moter1_BACK(duty)       GPIO_ResetBits(GPIOD, GPIO_Pin_3);TIM_SetCompare1(TIM8,duty)

#define Moter2_STOP  	          TIM_SetCompare2(TIM8,0)
#define Moter2_DISABLE          TIM_SetCompare2(TIM8,0)
#define Moter2_FORWARD(duty)    GPIO_SetBits(GPIOD, GPIO_Pin_4);TIM_SetCompare2(TIM8,duty)
#define Moter2_BACK(duty)       GPIO_ResetBits(GPIOD, GPIO_Pin_4);TIM_SetCompare2(TIM8,duty)

#define Moter3_STOP  	          TIM_SetCompare3(TIM8,0)
#define Moter3_DISABLE          TIM_SetCompare3(TIM8,0)
#define Moter3_FORWARD(duty)    GPIO_SetBits(GPIOD, GPIO_Pin_5);TIM_SetCompare3(TIM8,duty)
#define Moter3_BACK(duty)       GPIO_ResetBits(GPIOD, GPIO_Pin_5);TIM_SetCompare3(TIM8,duty)

#define Moter4_STOP  	          TIM_SetCompare4(TIM8,0)
#define Moter4_DISABLE          TIM_SetCompare4(TIM8,0)
#define Moter4_FORWARD(duty)    GPIO_SetBits(GPIOD, GPIO_Pin_6);TIM_SetCompare4(TIM8,duty)
#define Moter4_BACK(duty)       GPIO_ResetBits(GPIOD, GPIO_Pin_6);TIM_SetCompare4(TIM8,duty)
void TIM1_PWM_init(void);
void TIM8_PWM_init(void);
//TIM_SetCompare1(TIM1,0);          //修改占空比，第二个值为占空比，其取值范围从0到配置的TIM_Period的值，这里取800
//                                  //修改频率，
#endif





























