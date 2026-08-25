#include "hwt101ct.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "usart.h"

extern uint8_t UART3_RxRealLength;              //串口3每次接收指令的实际长度
extern uint8_t UART3_RxBuf[];    //串口3存放真实数据的数组（不包含包头包尾）

HWT101CT_DATA HWT101CT_Data;

/**
  * @brief 初始化陀螺仪
  */
void HWT101CT_Init(void){
  uint8_t buff1[5] = {0XFF, 0XAA, 0X69, 0X88, 0XB5};//解锁陀螺仪配置指令
  uint8_t buff2[5] = {0XFF, 0XAA, 0X76, 0X00, 0X00};//Z轴归零
	uint8_t buff3[5] = {0XFF, 0XAA, 0X00, 0XFF, 0X00};//重启

	// Usart_SendArray( USART3, &buff1[0], 5);     
  HAL_UART_Transmit(&huart3, buff1, 5, 100);//解锁陀螺仪配置指令
	HAL_Delay(200);
	// Usart_SendArray( USART3, &buff2[0], 5);     //Z轴归零
  HAL_UART_Transmit(&huart3, buff2, 5, 100);//Z轴归零
	HAL_Delay(200);
	// Usart_SendArray( USART3, &buff3[0], 5);     //重启
  HAL_UART_Transmit(&huart3, buff3, 5, 100);//重启
	HAL_Delay(200);

  HAL_Delay(300);//等待重启
}


/**
  * @brief 每次接收到陀螺仪数据后进行解析
  */
void HWT101CT_Update(void){
  if(((0xA8+UART3_RxBuf[15]+UART3_RxBuf[16]+UART3_RxBuf[17]+UART3_RxBuf[18])&0xFF)\
    ==UART3_RxBuf[21])                                                       //帧尾校验判断传输是否正确
  {
    HWT101CT_Data.yaw = ((float)((UART3_RxBuf[18]<<8)|UART3_RxBuf[17]))/32768*180;          //计算并更新角度值
  }

} 


