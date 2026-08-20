/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTI
  
  AL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"

#include "FreeRTOS.h"					//FreeRTOS使用		  
#include "task.h" 

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/
/**************************************************************************
函数功能：配置USART1的中断
输入变量：无
返回值	：无
说明    ：1、根据手册，IDLE的中断标志位的清除需要软件序列：
             先读寄存器SR再度寄存器DR，即先后调用函数
					   USART_GetITStatus(USART2, USART_IT_IDLE);
		         USART_ReceiveData(USART2);
**************************************************************************/
#include "protocol.h"
#include "usart.h"
#include "Oled.h"
#include "openMV.h"
extern volatile char scannerData; 
void cmd_data_recv(uint8_t *data, uint16_t data_len);
extern volatile uint32_t protocol_raw_data[128];                                            //陀螺仪原始数据
extern volatile uint8_t protocol_data[128];                                            //陀螺仪原始数据
uint8_t usart1_dma_data_len = 0;
//#define PROTOCOL_USART	//使用串口助手，注释掉则使用串口通信与主控连接
void USART1_IRQHandler(void)
{	
	if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET)
	{	
		MVData2[0] = USART_ReceiveData(USART1);
    //USART_SendData(UART5,MVData2);    
	}	 
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
//#ifdef PROTOCOL_USART	
//		if(USART_GetITStatus(USART1, USART_IT_IDLE)){
//		  USART_ReceiveData(USART1);
//			usart1_dma_data_len = 128 - DMA1_Channel5->CNDTR;
//			for(int i=0;i<usart1_dma_data_len;i++)
//			{
//				protocol_data[i] = (uint8_t)(protocol_raw_data[i]&0xFF);
//				Usart_SendByte( USART1, (uint8_t)protocol_data[i]);
//			}
//			protocol_data_recv((uint8_t*)protocol_data, usart1_dma_data_len);
//			receiving_process();
//			DMA_Cmd(DMA1_Channel5 , DISABLE);                                     //关闭DMA1通道6
//			DMA1_Channel5->CNDTR=128; 																						  //重新设置传输数量
//			DMA_Cmd(DMA1_Channel5 , ENABLE);                                      //开启DMA1通道6
//	}
//									u8g2_ClearBuffer(&u8g2);
//										oled_print_float( 10,10,(float)( 1));
//										u8g2_SendBuffer(&u8g2);	
//#endif
//			if(USART_GetITStatus(USART1, USART_IT_IDLE)){
//		  USART_ReceiveData(USART1);
//			scannerData	= protocol_data[0];

//			DMA_Cmd(DMA1_Channel5 , DISABLE);                                     //关闭DMA1通道6
//			DMA1_Channel5->CNDTR=128; 																						  //重新设置传输数量
//			DMA_Cmd(DMA1_Channel5 , ENABLE);                                      //开启DMA1通道6
//	}
//			scannerData	= protocol_data[0];
}

/**************************************************************************
函数功能：配置USART2的中断
输入变量：无
返回值	：无
说明    ：1、根据手册，IDLE的中断标志位的清除需要软件序列：
             先读寄存器SR再度寄存器DR，即先后调用函数
					   USART_GetITStatus(USART2, USART_IT_IDLE);
		         USART_ReceiveData(USART2);
					2、判断是否传输正确，如果出错
					   则说明通信受到干扰，此时DMA传输会一直错位，因此需要重新设置
						 DMA传输，让其重新接收新的一帧数据
**************************************************************************/
#include "HWT101CT_TTL.h"
#include "usart.h"
void USART3_IRQHandler(void)
{
	if(USART_GetITStatus(USART3, USART_IT_IDLE)){
		USART_ReceiveData(USART3);                                              //清除中断标志位
		//Usart_SendByte( USART3, (uint8_t)raw_data[0]);
		if(((0xA8+raw_data[15]+raw_data[16]+raw_data[17]+raw_data[18])&0xFF)\
			==raw_data[21])                                                       //帧尾校验判断传输是否正确
		{
			//w_z = ((float)((raw_data[7]<<8)|raw_data[6])) / 32768 * 2000;
			angle = ((float)((raw_data[18]<<8)|raw_data[17]))/32768*180;          //计算并更新角度值
			//Usart_SendByte( USART1, (uint8_t)angle);
		}
		else
		{
			DMA_Cmd(DMA1_Channel3 , DISABLE);                                     //关闭DMA1通道6
			DMA1_Channel3->CNDTR=22; 																						  //重新设置传输数量
			DMA_Cmd(DMA1_Channel3 , ENABLE);                                      //开启DMA1通道6
		}
	}
}
   
/**************************************************************************
函数功能：配置USART4的中断,蓝牙接收
输入变量：无
返回值	：无
说明    ：蓝牙数据自动接收，并存放在BlueData中
**************************************************************************/
#include "Blue_HC05.h"
void UART4_IRQHandler(void)
{
	if(USART_GetITStatus(UART4,USART_IT_RXNE)!=RESET)
	{		
		BlueData = USART_ReceiveData(UART4);
    //USART_SendData(USART1,BlueData);    
	}	 
}


extern volatile char MVData[2]; //蓝牙数据
/**************************************************************************
函数功能：配置USART5的中断,openMV
输入变量：无
返回值	：无
说明    ：openMV数据自动接收，并存放在openMVData中
**************************************************************************/
#include "openMV.h"
void UART5_IRQHandler(void)
{
	if(USART_GetITStatus(UART5,USART_IT_RXNE)!=RESET)
	{	
		MVData[0] = USART_ReceiveData(UART5);
    //USART_SendData(UART5,openMVData);    
	}	 
		USART_ClearITPendingBit(UART5, USART_IT_RXNE);
}

extern volatile char MVData2[2]; //蓝牙数据
/**************************************************************************
函数功能：配置USART2的中断,openMV2
输入变量：无
返回值	：无
说明    ：openMV数据自动接收，并存放在openMVData中
**************************************************************************/
#include "openMV.h"
void USART2_IRQHandler(void)
{			
	if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET)
	{	
		MVData2[0] = USART_ReceiveData(USART2);
    //USART_SendData(UART5,openMVData);    
	}	 
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
}

/**************************************************************************
函数功能：定时器中断服务函数
输入变量：无
返回值	：无
说明    ：
**************************************************************************/
void  RunPIDTask_main(void);
void  TIM6_IRQHandler (void)
{
	if ( TIM_GetITStatus( TIM6, TIM_IT_Update) != RESET ) 
	{	
		RunPIDTask_main();
		TIM_ClearITPendingBit(TIM6 , TIM_FLAG_Update);  		 
	}		 	
}
/**************************************************************************
函数功能：定时器中断服务函数
输入变量：无
返回值	：无
说明    ：
**************************************************************************/
//extern int speed_count;
//void  TIM7_IRQHandler (void)
//{
//	if ( TIM_GetITStatus( TIM7, TIM_IT_Update) != RESET ) 
//	{	
//		speed_count ++;
//		TIM_ClearITPendingBit(TIM7 , TIM_FLAG_Update);  		 
//	}		 	
//}
/**************************************************************************
函数功能：定时器中断服务函数
输入变量：无
返回值	：无
说明    ：
**************************************************************************/
#include "pwm.h"
#include "delay.h"
#include "motor.h"
extern int encoder1_num;
extern int encoder2_num;
extern int encoder3_num;
extern int encoder4_num;

void  TIM2_IRQHandler (void)
{
	if ( TIM_GetITStatus( TIM2, TIM_IT_Update) != RESET ) 
	{	
		if((TIM2->CR1>>4&0x01)==0)//DIR=0
		{
			encoder1_num++;//计数加一
			if((encoder1_num*5120+TIM_GetCounter(TIM2)-my_car.motor_1.encoder_count_all_last)>3000)encoder1_num--;
		}
		else if((TIM2->CR1>>4&0x01)==1)//DIR=1
		{
			encoder1_num--;//计数加一
			if((encoder1_num*5120+TIM_GetCounter(TIM2)-my_car.motor_1.encoder_count_all_last)<-3000)encoder1_num++;
		}
//		Moter1_STOP;
//		delay_ms(20);
//		Moter1_DISABLE;
		 		 //Usart_SendByte( USART1, 0x11);Usart_SendByte( USART1, 0x11);
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update); 
	}		 	
}
void  TIM3_IRQHandler (void)
{
	if ( TIM_GetITStatus( TIM3, TIM_IT_Update) != RESET ) 
	{	
		if((TIM3->CR1>>4&0x01)==0)//DIR=0
		{
			encoder2_num++;//计数加一
			if((encoder2_num*5120+TIM_GetCounter(TIM3)-my_car.motor_2.encoder_count_all_last)>3000)encoder2_num--;
		}
		else if((TIM3->CR1>>4&0x01)==1)//DIR=1
		{
			encoder2_num--;//计数加一
			if((encoder2_num*5120+TIM_GetCounter(TIM3)-my_car.motor_2.encoder_count_all_last)<-3000)encoder2_num++;
		}
//		Moter2_STOP;
//		delay_ms(20);
//		Moter2_DISABLE;
		 		 //Usart_SendByte( USART1, 0x11);Usart_SendByte( USART1, 0x11);
		TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update); 
	}		 	
}
void  TIM4_IRQHandler (void)
{
	if ( TIM_GetITStatus( TIM4, TIM_IT_Update) != RESET ) 
	{	
		if((TIM4->CR1>>4&0x01)==0)//DIR=0
		{
			encoder3_num++;//计数加一
			if((encoder3_num*5120+TIM_GetCounter(TIM4)-my_car.motor_3.encoder_count_all_last)>3000)encoder3_num--;
		}
		else if((TIM4->CR1>>4&0x01)==1)//DIR=1
		{
			encoder3_num--;//计数加一
			if((encoder3_num*5120+TIM_GetCounter(TIM4)-my_car.motor_3.encoder_count_all_last)<-3000)encoder3_num++;
		}
//		Moter3_STOP;
//		delay_ms(20);
//		Moter3_DISABLE;
		 		 //Usart_SendByte( USART1, 0x11);Usart_SendByte( USART1, 0x11);
		TIM_ClearITPendingBit(TIM4 , TIM_FLAG_Update); 
	}		 	
}
void  TIM5_IRQHandler (void)
{
	if ( TIM_GetITStatus( TIM5, TIM_IT_Update) != RESET ) 
	{	
		if((TIM5->CR1>>4&0x01)==0)//DIR=0
		{
			encoder4_num++;//计数加一
			if((encoder4_num*5120+TIM_GetCounter(TIM5)-my_car.motor_4.encoder_count_all_last)>3000)encoder4_num--;
		}
		else if((TIM5->CR1>>4&0x01)==1)//DIR=1
		{
			encoder4_num--;//计数加一
			if((encoder4_num*5120+TIM_GetCounter(TIM5)-my_car.motor_4.encoder_count_all_last)<-3000)encoder4_num++;
		}
//		Moter4_STOP;
//		delay_ms(20);
//		Moter4_DISABLE;
		 		 //Usart_SendByte( USART1, 0x11);Usart_SendByte( USART1, 0x11);
		TIM_ClearITPendingBit(TIM5 , TIM_FLAG_Update); 
	}		 	
}
/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
//void SVC_Handler(void)
//{
//}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//}

///**
//  * @brief  This function handles SysTick Handler.
//  * @param  None
//  * @retval None
//  */
extern void xPortSysTickHandler(void);
//systick中断服务函数
void SysTick_Handler(void)
{	
    #if (INCLUDE_xTaskGetSchedulerState  == 1 )
      if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
      {
    #endif  /* INCLUDE_xTaskGetSchedulerState */  
        xPortSysTickHandler();
    #if (INCLUDE_xTaskGetSchedulerState  == 1 )
      }
    #endif  /* INCLUDE_xTaskGetSchedulerState */
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
