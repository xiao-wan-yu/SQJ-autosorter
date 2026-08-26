/*-------------------------------文件说明--------------------------------*/
/*	本文件未经作者允许不得用于商业用途
	作者：枭白
	版本：v1.0
	日期：2022.7.15
*/	
/*-------------------------------文件包含--------------------------------*/

#include "dma.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_dma.h"
#include "HWT101CT_TTL.h"
/*-------------------------------全局变量--------------------------------*/




/*-------------------------------函数定义--------------------------------*/

/**************************************************************************
函数功能：配置USART1的DMA
输入变量：无
返回值	：无
说明    ：1、不同的外设对应不同的DMA通道，USRAT2的RX对应DMA1通道6
          2、虽然串口的数据寄存器只有9位有效，并且通常只用低八位，
					   但是数据的大小还是需要设置成32位
					3、使用DMA进行数据传输时，如果数据量小，则可以传输完成后对数据进行读取使用
					   但是当传输的数据量比较大，并且两次传输的时间较短，则最好DMA传输一半时读取
						 然后再传输另一半再读取，避免出错。
					4、DMA与CPU对存储器的访问时交替的，内部有仲裁器，因此在DMA传输的时候可以直接程序读取数据，
					   但读取与DMA传输总有一个会挂起，因为他们对总线的控制是轮换的。
**************************************************************************/
void DMA_Config(void)
{
	DMA_InitTypeDef DMA_InitStruct;																						//定义初始化结构体
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);												//开启DMA1时钟
	
	DMA_DeInit(DMA1_Channel6);																								//复位DMA1——Channel6
	DMA_InitStruct.DMA_BufferSize=22;																          //传输的数据量，即传输次数
	DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralSRC;															//外设到存储器
	DMA_InitStruct.DMA_M2M=DMA_M2M_Disable;																		//关闭存储器到存储器模式
	DMA_InitStruct.DMA_MemoryBaseAddr=( uint32_t )raw_data;									  //存储器基地址为数组首地址，即数组名
	DMA_InitStruct.DMA_MemoryDataSize=DMA_MemoryDataSize_Word;								//存储器的数据长度类型是32位
	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;												//存储器地址增量模式
	DMA_InitStruct.DMA_Mode=DMA_Mode_Circular;																//不设置为循环模式
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)(&(USART2->DR));				  //外设的基地址
	DMA_InitStruct.DMA_PeripheralDataSize=DMA_MemoryDataSize_Word;				    //外设的数据长度类型是32位
	DMA_InitStruct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;								//外设地址不增加
	DMA_InitStruct.DMA_Priority=DMA_Priority_VeryHigh;												//DMA通道优先级为最高
	
	//DMA_ITConfig(DMA1_Channel6, DMA_IT_TC, ENABLE);														//传输完成产生中断
	
	DMA_Init(DMA1_Channel6,&DMA_InitStruct);																	//初始化DMA1通道6 （未使能，即还没开启通道）
	DMA_Cmd(DMA1_Channel6 , ENABLE);                                          //开启DMA1通道1
}
