/*-------------------------------使用说明--------------------------------*/
/**************************************************************************
使用时将u8g2文件夹添加到工程目录下，然后添加所有c文件到工程

然后再引用头文件Oled.h,并使用Oled_Init();初始化即可使用

使用的是i2c通讯，端口为PB6-SCL，PB7-SDA

原文：https://www.bilibili.com/read/cv15875042
**************************************************************************/
#ifndef _OLED_H_
#define _OLED_H_

/*-------------------------------文件包含--------------------------------*/
#include "stm32f10x.h"
#include "u8g2.h"
#include "delay.h"
#include <stdio.h>
#include <string.h>
/*-------------------------------引脚定义--------------------------------*/

#define SCL_Pin GPIO_Pin_10                                                  //软件i2c的SCL
#define SDA_Pin GPIO_Pin_11                                                 //软件i2c的SDA
#define IIC_GPIO_Port GPIOB

extern	u8g2_t u8g2;

void oled_print_float(u8g2_uint_t x,u8g2_uint_t y,float value);
void Oled_Init(void);
	
#endif 


