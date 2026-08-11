

/*-------------------------------文件说明--------------------------------*/
/*	本文件未经作者允许不得用于商业用途
	作者：枭白
	版本：v1.0
	日期：2022.7.15
*/	
/*-------------------------------文件包含--------------------------------*/

#include "NVIC.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

/*-------------------------------全局变量--------------------------------*/





/*-------------------------------函数定义--------------------------------*/

/**************************************************************************
函数功能：初始化DMA1的中断向量控制器
输入变量：无
返回值	：无
**************************************************************************/
void NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStruct;
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStruct.NVIC_IRQChannel=DMA1_Channel6_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=1;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	
	NVIC_Init(&NVIC_InitStruct);
}

