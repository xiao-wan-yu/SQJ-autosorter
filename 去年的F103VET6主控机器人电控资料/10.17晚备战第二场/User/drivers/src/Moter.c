#include "Moter.h"

/**************************************************************************
接线：
ST   ---------------------PD3
FRA  ---------------------PD4
STEPA---------------------PD5
FRB  ---------------------PD6
STEPB---------------------PD7
FRC  ---------------------PB3
STEPC---------------------PB4
FRD  ---------------------PB5
STEPD---------------------PB9

传感器调试：

**************************************************************************/
#include "stm32f10x.h" //寄存器映射与宏定义
#include "gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "delay.h"
/**************************************************************************
函数功能：配置GPIO，相关结构体、参数枚举、宏定义等在stm32f10x_gpio.h
输入变量：无
返回值	：无
说明    ：配置GPIO相关结构体、参数枚举、宏定义等在stm32f10x_gpio.h
					外设的时钟在APB2总线上，开启时钟的相关函数参数在stm32f10x_rcc.h
**************************************************************************/
static void Moter_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;//初始化结构体
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//根据需要选择输出模式
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_9;//输出引脚号
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;//配置输出速度，跟功耗有关，速度越高，功耗越高
	GPIO_Init(GPIOB,&GPIO_InitStruct);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//根据需要选择输出模式
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;//输出引脚号
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;//配置输出速度，跟功耗有关，速度越高，功耗越高
	GPIO_Init(GPIOD,&GPIO_InitStruct);
	
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//根据需要选择输出模式
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_12;//输出引脚号
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;//配置输出速度，跟功耗有关，速度越高，功耗越高
	GPIO_Init(GPIOD,&GPIO_InitStruct);
	
	
}
void Moter_init(void)
{
  Moter_GPIO_Config();
	Moter_ENABLE;
	Moter_FRA_F;
	Moter_FRD_F;
}


void Forward (unsigned int a,unsigned char gear)							//a 为步数,gear 为挡位
{
	
	unsigned char sp = 60-10*gear;														  //挡位换算
	Moter_FRA_F;
	Moter_FRD_F;
		for (int i=0;i<a;i++)
	{
			Moter_STEPA_H;
			Moter_STEPD_H;
			delay_us(sp);
			Moter_STEPA_L;
			Moter_STEPD_L;
			delay_us(sp);
	}
}

void Back (unsigned int a,unsigned char gear)							//a 为步数，gear 为挡位
{
	
	unsigned char sp = 60-10*gear;
	Moter_FRA_R;
	Moter_FRD_R;
		for (int i=0;i<a;i++)
	{
			Moter_STEPA_H;
			Moter_STEPD_H;
			delay_us(sp);
			Moter_STEPA_L;
			Moter_STEPD_L;
			delay_us(sp);
	}
}

void Left (unsigned int a,unsigned char gear)							//a = 7990为360度 3960为180度 1980为90度	990为45度
{
	
	unsigned char sp = 60-10*gear;
	Moter_FRA_R;
	Moter_FRD_F;
		for (int i=0;i<a;i++)
	{
			Moter_STEPA_H;
			Moter_STEPD_H;
			delay_us(sp);
			Moter_STEPA_L;
			Moter_STEPD_L;
			delay_us(sp);
	}
}


void Right (unsigned int a,unsigned char gear)
{
	
	unsigned char sp = 60-10*gear;
	Moter_FRA_F;
	Moter_FRD_R;
		for (int i=0;i<a;i++)
	{
			Moter_STEPA_H;
			Moter_STEPD_H;
			delay_us(sp);
			Moter_STEPA_L;
			Moter_STEPD_L;
			delay_us(sp);
	}
}








