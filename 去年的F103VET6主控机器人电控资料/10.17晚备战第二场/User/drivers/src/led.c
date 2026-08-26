#include "led.h"

/**************************************************************************
接线：
LEDRH----EXIO10
LEDRL----EXIO11
LEDGH----EXIO13
LEDGL----EXIO14

传感器调试：

**************************************************************************/

#include "stm32f10x.h" //寄存器映射与宏定义
#include "gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
/**************************************************************************
函数功能：配置GPIO，相关结构体、参数枚举、宏定义等在stm32f10x_gpio.h
输入变量：无
返回值	：无
说明    ：配置GPIO相关结构体、参数枚举、宏定义等在stm32f10x_gpio.h
					外设的时钟在APB2总线上，开启时钟的相关函数参数在stm32f10x_rcc.h
**************************************************************************/
void LED_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;//初始化结构体
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_5|\
	                           GPIO_Pin_6;//引脚号
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE,&GPIO_InitStruct);
	
	GPIO_ResetBits(GPIOE,GPIO_Pin_2);
	GPIO_ResetBits(GPIOE,GPIO_Pin_5);
	
}























