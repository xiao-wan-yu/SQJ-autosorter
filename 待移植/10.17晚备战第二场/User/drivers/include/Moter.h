#ifndef __MOTER_H
#define __MOTER_H

/*-------------------------------文件包含--------------------------------*/
#include "stm32f10x.h"
/*-------------------------------DRV8825请使用--------------------------------*/
#if 0
//电机驱动板使能
#define Moter_ENABLE     GPIO_ResetBits(GPIOD, GPIO_Pin_3)//使能，电机供电，停住
#define Moter_DISABLE    GPIO_SetBits(GPIOD, GPIO_Pin_3)//禁用，电机不供电，电机松弛
//电机A控制
#define Moter_FRA_F      GPIO_ResetBits(GPIOD, GPIO_Pin_4)//顺时针转,正看转轴
#define Moter_FRA_R      GPIO_SetBits(GPIOD, GPIO_Pin_4)//逆时针转,正看转轴
#define Moter_STEPA_H    GPIO_SetBits(GPIOD, GPIO_Pin_5)//PWM调步
#define Moter_STEPA_L    GPIO_ResetBits(GPIOD, GPIO_Pin_5)//
//电机B控制
#define Moter_FRB_F      GPIO_ResetBits(GPIOD, GPIO_Pin_6)//顺时针转,正看转轴
#define Moter_FRB_R      GPIO_SetBits(GPIOD, GPIO_Pin_6)//逆时针转,正看转轴
#define Moter_STEPB_H    GPIO_SetBits(GPIOD, GPIO_Pin_7)//PWM调步
#define Moter_STEPB_L    GPIO_ResetBits(GPIOD, GPIO_Pin_7)//
#endif
/*-------------------------------LV8731请使用--------------------------------*/
#if 1
//电机驱动板使能
#define Moter_ENABLE     GPIO_SetBits(GPIOD, GPIO_Pin_3)//使能，电机供电，停住
#define Moter_DISABLE    GPIO_ResetBits(GPIOD, GPIO_Pin_3)////禁用，电机不供电，电机松弛
//电机A控制
#define Moter_FRA_F      GPIO_SetBits(GPIOD, GPIO_Pin_4)//顺时针转,正看转轴
#define Moter_FRA_R      GPIO_ResetBits(GPIOD, GPIO_Pin_4)//逆时针转,正看转轴			 正走
#define Moter_STEPA_H    GPIO_SetBits(GPIOD, GPIO_Pin_5)//PWM调步
#define Moter_STEPA_L    GPIO_ResetBits(GPIOD, GPIO_Pin_5)//
//电机B控制
#define Moter_FRB_F      GPIO_ResetBits(GPIOD, GPIO_Pin_6)//顺时针转,正看转轴				 正走
#define Moter_FRB_R      GPIO_SetBits(GPIOD, GPIO_Pin_6)//逆时针转,正看转轴
#define Moter_STEPB_H    GPIO_SetBits(GPIOD, GPIO_Pin_7)//PWM调步
#define Moter_STEPB_L    GPIO_ResetBits(GPIOD, GPIO_Pin_7)//
//电机C控制
#define Moter_FRC_F      GPIO_SetBits(GPIOB, GPIO_Pin_3)//顺时针转,正看转轴
#define Moter_FRC_R      GPIO_ResetBits(GPIOB, GPIO_Pin_3)//逆时针转,正看转轴
#define Moter_STEPC_H    GPIO_SetBits(GPIOB, GPIO_Pin_4)//PWM调步
#define Moter_STEPC_L    GPIO_ResetBits(GPIOB, GPIO_Pin_4)//
//电机D控制
#define Moter_FRD_F      GPIO_ResetBits(GPIOB, GPIO_Pin_5)//顺时针转,正看转轴
#define Moter_FRD_R      GPIO_SetBits(GPIOB, GPIO_Pin_5)//逆时针转,正看转轴
#define Moter_STEPD_H    GPIO_SetBits(GPIOB, GPIO_Pin_9)//PWM调步
#define Moter_STEPD_L    GPIO_ResetBits(GPIOB, GPIO_Pin_9)//




#define Moter_FRE_F      GPIO_ResetBits(GPIOD, GPIO_Pin_13)//顺时针转,正看转轴				 正走
#define Moter_FRE_R      GPIO_SetBits(GPIOD, GPIO_Pin_13)//逆时针转,正看转轴
#define Moter_STEPE_H    GPIO_SetBits(GPIOD, GPIO_Pin_12)//PWM调步
#define Moter_STEPE_L    GPIO_ResetBits(GPIOD, GPIO_Pin_12)//
#endif


void Moter_init(void);


void Forward (unsigned int a,unsigned char gear);
void Left (unsigned int a,unsigned char gear);
void Right (unsigned int a,unsigned char gear);
void Back (unsigned int a,unsigned char gear);
	
#endif





























