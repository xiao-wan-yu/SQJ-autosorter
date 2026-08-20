#include "8LuHuiDu.h"

/**************************************************************************
接线：
OUT1---------------------PE13
OUT2---------------------PE14
OUT3---------------------PE11
OUT4---------------------PE12
OUT5---------------------PE9
OUT6---------------------PE10
OUT7---------------------PE7
OUT8---------------------PE8

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
void HuiDu_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;//初始化结构体
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空数字输入
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_13|GPIO_Pin_12|\
	                           GPIO_Pin_11|GPIO_Pin_10|GPIO_Pin_9|\
	                           GPIO_Pin_8|GPIO_Pin_7;//引脚号
	GPIO_Init(GPIOE,&GPIO_InitStruct);
	
	
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空数字输入
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14;//引脚号
	GPIO_Init(GPIOD,&GPIO_InitStruct);
	
	
}























