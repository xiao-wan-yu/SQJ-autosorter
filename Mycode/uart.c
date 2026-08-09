#include <stdint.h>
#include <main.h>
#include <string.h>//用于字符串处理的库
#include <stdarg.h>
#include <stdlib.h>
#include "stdio.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_uart.h"
#include "uart.h"


extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;

uint8_t UART1_RxNewData;                //串口1最新接收到的数据
uint8_t UART1_RxBuf[UART1_RxLength];    //串口1存放真实数据的数组（不包含包头包尾）
uint8_t UART1_RxFlag = 0;               //串口1接收完成标志位（接收完成则为1）
uint8_t UART2_RxNewData;                //串口2最新接收到的数据
uint8_t UART2_RxBuf[UART2_RxLength];    //串口2存放真实数据的数组（不包含包头包尾）
uint8_t UART2_RxFlag = 0;               //串口2接收完成标志位（接收完成则为1）
uint8_t UART3_RxNewData;                //串口3最新接收到的数据
uint8_t UART3_RxBuf[UART3_RxLength];    //串口3存放真实数据的数组（不包含包头包尾）
uint8_t UART3_RxFlag = 0;               //串口3接收完成标志位（接收完成则为1）
uint8_t UART5_RxNewData;                //串口5最新接收到的数据
uint8_t UART5_RxBuf[UART5_RxLength];    //串口5存放真实数据的数组（不包含包头包尾）
__IO uint8_t UART5_RxFlag;                   //串口5接收完成标志位（接收完成则为1）

#if UART1_USE_DMA
uint8_t UART1_RxRealLength;              //串口1每次接收指令的实际长度
#endif

#if UART2_USE_GY53
uint16_t GY53_Distance = 0;             //GY53模块返回的测量距离，单位：mm
#endif

#if UART5_USE_SteppingMotor
uint8_t STEP_RxRealLength;              //步进电机每次接收指令的实际长度
#endif

/**
  * @brief 调试串口
  * @attention 调用方法：UART1_printf("123"); 需要以hex模式发送数据时，可先定义一个数组，再把数组名转换为char *类型即可
  */
void UART1_Printf(char *fmt, ...){
    char buff[UART1_TxLengthMax+1];  //用于存放转换后的数据 [长度]
    uint16_t i=0;
    va_list arg_ptr;
    va_start(arg_ptr, fmt);
    vsnprintf(buff, UART1_TxLengthMax+1, fmt,  arg_ptr);//数据转换
    i=strlen(buff);//得出数据长度
    if(strlen(buff)>UART1_TxLengthMax)i=UART1_TxLengthMax;//如果长度大于最大值，则长度等于最大值（多出部分忽略）
    HAL_UART_Transmit(&huart1,(uint8_t  *)buff,i,0xffff);//串口发送函数（串口号，内容，数量，溢出时间）
    va_end(arg_ptr);
}

/**
  * @brief 普通串口/测距模块通信串口
  * @attention 调用方法：UART2_printf("123"); 需要以hex模式发送数据时，可先定义一个数组，再把数组名转换为char *类型即可
  */
void UART2_Printf(char *fmt, ...){
    char buff[UART2_TxLengthMax+1];  //用于存放转换后的数据 [长度]
    uint16_t i=0;
    va_list arg_ptr;
    va_start(arg_ptr, fmt);
    vsnprintf(buff, UART2_TxLengthMax+1, fmt,  arg_ptr);//数据转换
    i=strlen(buff);//得出数据长度
    if(strlen(buff)>UART2_TxLengthMax)i=UART2_TxLengthMax;//如果长度大于最大值，则长度等于最大值（多出部分忽略）
    HAL_UART_Transmit(&huart2,(uint8_t  *)buff,i,0xffff);//串口发送函数（串口号，内容，数量，溢出时间）
    va_end(arg_ptr);
}

/**
  * @brief 普通串口/蓝牙模块通信串口
  * @attention 调用方法：UART3_printf("123"); 需要以hex模式发送数据时，可先定义一个数组，再把数组名转换为char *类型即可
  */
void UART3_Printf(char *fmt, ...){
    char buff[UART3_TxLengthMax+1];  //用于存放转换后的数据 [长度]
    uint16_t i=0;
    va_list arg_ptr;
    va_start(arg_ptr, fmt);
    vsnprintf(buff, UART3_TxLengthMax+1, fmt,  arg_ptr);//数据转换
    i=strlen(buff);//得出数据长度
    if(strlen(buff)>UART3_TxLengthMax)i=UART3_TxLengthMax;//如果长度大于最大值，则长度等于最大值（多出部分忽略）
    HAL_UART_Transmit(&huart3,(uint8_t  *)buff,i,0xffff);//串口发送函数（串口号，内容，数量，溢出时间）
    va_end(arg_ptr);
}

/**
  * @brief 串口接收中断回调函数模板
  *
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  static uint8_t UART1_RxState = 0;     //串口1接收状态
  static uint8_t UART1_RxBufIdxNow = 0; //串口1接收数组下标当前值
  // static uint8_t UART2_RxState = 0;     //串口2接收状态
  // static uint8_t UART2_RxBufIdxNow = 0; //串口2接收数组下标当前值
  static uint8_t UART3_RxState = 0;     //串口3接收状态
  static uint8_t UART3_RxBufIdxNow = 0; //串口3接收数组下标当前值

  if(huart == &huart1){//调试串口
    /*此处放置状态机*/
    /*模板状态机：接收包头为0xFF 包尾为0xFE的数据包*/
    switch(UART1_RxState){//接收包头包尾
      case 0://寻找包头
        if(UART1_RxNewData == 0xFF){//找到包头
          UART1_RxBufIdxNow = 0;
          UART1_RxState = 1;
        }
        break;
      case 1://采集数据
        UART1_RxBuf[UART1_RxBufIdxNow++] = UART1_RxNewData;
        if(UART1_RxBufIdxNow >= UART1_RxLength){//已经接收了UART1_RxLength个数据
          UART1_RxState = 2;
        }
        break;
      case 2://寻找包尾
        if(UART1_RxNewData == 0xFE){//找到包尾
          UART1_RxState = 0;
          UART1_RxFlag = 1;
        }
        break;
      default:
        break;
    }
    /*再次开启中断接收数据*/
    HAL_UART_Receive_IT(&huart1, &UART1_RxNewData, 1);
  }


  if(huart == &huart2){//普通串口/测距模块通信串口
    /*此处放置状态机*/

    /*再次开启中断接收数据*/
    HAL_UART_Receive_IT(&huart2, &UART2_RxNewData, 1);
  }


  if(huart == &huart3){//普通串口/蓝牙模块通信串口
    /*此处放置状态机*/
    /*模板状态机：接收包头为0xFF 包尾为0xFE的数据包*/
    switch(UART3_RxState){//接收包头包尾
      case 0://寻找包头
        if(UART3_RxNewData == 0xFF){//找到包头
          UART3_RxBufIdxNow = 0;
          UART3_RxState = 1;  
        }
        break;
      case 1://采集数据
        UART3_RxBuf[UART3_RxBufIdxNow++] = UART3_RxNewData;
        if(UART3_RxBufIdxNow >= UART3_RxLength){//已经接收了UART1_RxLength个数据
          UART3_RxState = 2;
        }
        break;
      case 2://寻找包尾
        if(UART3_RxNewData == 0xFE){//找到包尾
          UART3_RxState = 0;
          UART3_RxFlag = 1;
        }
        break;
      default:
        break;
    }
    /*再次开启中断接收数据*/
    HAL_UART_Receive_IT(&huart3, &UART3_RxNewData, 1);
  }
}



    // /*模板状态机：接收包头为0xFF 包尾为0xFE的数据包*/
    // switch(UART1_RxState){//接收包头包尾
    //   case 0://寻找包头
    //     if(UART1_RxNewData == 0xFF){//找到包头
    //       UART1_RxBufIdxNow = 0;
    //       UART1_RxState = 1;
    //     }
    //     break;
    //   case 1://采集数据
    //     UART1_RxBuf[UART1_RxBufIdxNow++] = UART1_RxNewData;
    //     if(UART1_RxBufIdxNow >= UART1_RxLength){//已经接收了UART1_RxLength个数据
    //       UART1_RxState = 2;
    //     }
    //     break;
    //   case 2://寻找包尾
    //     if(UART1_RxNewData == 0xFE){//找到包尾
    //       UART1_RxState = 0;
    //       UART1_RxFlag = 1;
    //     }
    //     break;
    //   default:
    //     break;
    // }

    // /*测距模块GY53状态机：接收包头为0x5A 0x5A 没有包尾 真实数据6个的数据包*/
    // switch(GY53_RxState){
    //   case 0:
    //     if(GY53_RxNewData == 0x5A){
    //       GY53_RxState = 1;
    //     }
    //     break;
    //   case 1:
    //     if(GY53_RxNewData == 0x5A){
    //       GY53_RxState = 2;
    //     }else{
    //       GY53_RxState = 0;
    //     }
    //     break;
    //   case 2:
    //     GY53_RxBuf[GY53_RxBufIdxNow++] = GY53_RxNewData;
    //     if(GY53_RxBufIdxNow >= 6){   //接收真实数据完毕
    //       if(GY53_RxBuf[5] == (uint8_t)(0x5A + 0x5A + GY53_RxBuf[0] + GY53_RxBuf[1] + 
    //         GY53_RxBuf[2] + GY53_RxBuf[3] + GY53_RxBuf[4])){//数据验证正确
    //           GY53_Distance = (GY53_RxBuf[2]<<8) | GY53_RxBuf[3];//更新GY53_Distance数据
    //           GY53_RxFlag = 1;
    //         }
    //       GY53_RxState = 0;
    //       GY53_RxBufIdxNow = 0;
    //     }
    //     break;
    //   default:
    //     break;
    // }



void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
  if(huart == &huart1){//调试串口
    /*此处进行数据处理*/
    /*对调试串口返回的数据进行处理*/
    UART1_RxRealLength = Size;
    UART1_RxFlag = 1;

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART1_RxBuf, UART1_RxLength);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);    //使用DMA+UART时，会开启传输过半中断，需手动关闭
  }
  
  if(huart == &huart5){//普通串口/步进电机串口
    /*此处进行数据处理*/
    /*对步进电机返回的数据进行处理*/
    STEP_RxRealLength = Size;
    STEP_RxFlag = 1;

    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, STEP_RxBuf, STEP_RxLength);
    __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);    //使用DMA+UART时，会开启传输过半中断，需手动关闭
  }

}

