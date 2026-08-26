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
void GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;//初始化结构体
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//根据需要选择输出模式
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;//输出引脚号
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;//配置输出速度，跟功耗有关，速度越高，功耗越高
	GPIO_Init(GPIOC,&GPIO_InitStruct);
}

static void gpio_output()//无用函数，介绍IO输出的用法
{
		GPIO_SetBits(GPIOC, GPIO_Pin_13);//输出高电平
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);//输出低电平
}

static void gpio_input()//无用函数，介绍IO输入的用法
{
		GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13);//IO电平读取，高或者低
}
