/*-------------------------------文件说明--------------------------------*/
/*	本文件未经作者允许不得用于商业用途
	作者：枭白
	版本：v1.0
	日期：2022.7.15
*/	
/*-------------------------------文件包含--------------------------------*/

#include "bsp_adc.h"
#include "delay.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_gpio.h"

/*-------------------------------全局变量--------------------------------*/





/*-------------------------------函数定义--------------------------------*/
/**************************************************************************
函数功能：配置DMA1的通道1
输入变量：无
返回值	：无
**************************************************************************/
uint32_t ADC_Data[3]={0};																				//ADC采集数据
uint8_t Data_Collect_State=0;                                               //数据采集状态标志位
static void DMA1_Channel_1_Config(void)
{
	DMA_InitTypeDef DMA_InitStruct;																						//定义初始化结构体
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);												//开启DMA1时钟
	
	DMA_DeInit(DMA1_Channel1);																								//复位DMA1——Channel1
	DMA_InitStruct.DMA_BufferSize=3;																          //传输的数据量，即传输次数
	DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralSRC;															//外设到存储器
	DMA_InitStruct.DMA_M2M=DMA_M2M_Disable;																		//关闭存储器到存储器模式
	DMA_InitStruct.DMA_MemoryBaseAddr=( uint32_t )ADC_Data;									  //存储器基地址为数组首地址，即数组名
	DMA_InitStruct.DMA_MemoryDataSize=DMA_PeripheralDataSize_Word;								//存储器的数据长度类型是32位
	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;												//存储器地址增量模式
	DMA_InitStruct.DMA_Mode=DMA_Mode_Circular;																	//不设置为循环模式
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)(&(ADC1->DR));						//外设的基地址为ADC1的DR寄存器地址
	DMA_InitStruct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Word;				//外设的数据长度类型是32位
	DMA_InitStruct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;								//外设地址不增加
	DMA_InitStruct.DMA_Priority=DMA_Priority_VeryHigh;												//DMA通道优先级为最高
	
	//DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);														//传输完成产生中断
	
	DMA_Init(DMA1_Channel1,&DMA_InitStruct);																	//初始化DMA1通道1 （未使能，即还没开启通道）
	DMA_Cmd(DMA1_Channel1 , ENABLE);    
}
/**************************************************************************
函数功能：配置ADC1工作模式
输入变量：无
返回值	：无
**************************************************************************/
void ADC1_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;//初始化ADC采集IO
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStruct.GPIO_Pin =GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AIN;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	DMA1_Channel_1_Config();
	
	ADC_InitTypeDef ADC_InitStructure;	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);											//开启APB2时钟
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE ; 
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	
	ADC_InitStructure.ADC_NbrOfChannel = 3;	
	ADC_Init(ADC1, &ADC_InitStructure);
	
	
	// 配置 ADC 通道转换顺序和采样时间
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 2, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 3, ADC_SampleTime_55Cycles5);
	
	// ADC 转换结束产生中断，在中断服务程序中读取转换值

	ADC_DMACmd(ADC1,ENABLE);																									//使能DMA请求
	
	// 开启ADC ，并开始转换
	ADC_Cmd(ADC1, ENABLE);
	
	// 初始化ADC 校准寄存器  
	ADC_ResetCalibration(ADC1);
	// 等待校准寄存器初始化完成
	while(ADC_GetResetCalibrationStatus(ADC1));
	
	// ADC开始校准
	ADC_StartCalibration(ADC1);
	// 等待校准完成
	while(ADC_GetCalibrationStatus(ADC1));
	
	// 由于没有采用外部触发，所以使用软件触发ADC转换 
	//ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	
}
int aq=0;
#include "Oled.h"
void ADC_read(void)
{
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while(!(ADC1->SR&0x02));
	delay_ms(10);
	u8g2_ClearBuffer(&u8g2);
	oled_print_float( 10,10,(float)( ADC_Data[0]&0xFFF));
	oled_print_float( 10,20,(float)( ADC_Data[1]&0xFFF));
	oled_print_float( 10,30,(float)( ADC_Data[2]&0xFFF));
	u8g2_SendBuffer(&u8g2);	
}

