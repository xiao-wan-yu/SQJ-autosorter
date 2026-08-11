#ifndef __UART_H
#define __UART_H

#include <stm32f4xx_hal.h>

//串口1开启DMA接收时置1
#define UART1_USE_DMA               1
//使用测距模块GY53时置1 不使用测距模块GY53时置0（连续模式+串口中断接收）
#define UART2_USE_GY53              0
//使用蓝牙模块时置1 不使用蓝牙模块时置0
#define UART3_USE_BlueTeeth         0
//使用步进电机时置1 不使用步进电机时置0
#define UART5_USE_SteppingMotor     1


#define UART1_RxLength 200                  //串口1接收数据包的真实数据长度（开启DMA接收时，则为最大接收长度）
#define UART1_TxLengthMax 200               //串口1发送数据的最大数据长度
#define UART2_RxLength 6                    //串口2接收数据包的真实数据长度
#define UART2_TxLengthMax 200               //串口2发送数据的最大数据长度
#define UART3_RxLength 2                    //串口3接收数据包的真实数据长度
#define UART3_TxLengthMax 200               //串口3发送数据的最大数据长度
#define UART4_TxLengthMax 200               //串口4(备用串口)发送数据的最大数据长度
#define UART5_RxLength 255                  //串口5接收数据包的真实数据长度（开启DMA接收时，则为最大接收长度）
#define UART5_TxLengthMax 200               //串口5发送数据的最大数据长度

extern uint8_t UART1_RxNewData;             //串口1最新接收到的数据
extern uint8_t UART1_RxBuf[];               //串口1存放真实数据的数组（不包含包头包尾）
extern uint8_t UART1_RxFlag;                //串口1接收完成标志位（接收完成则为1）
extern uint8_t UART2_RxNewData;             //串口2最新接收到的数据
extern uint8_t UART2_RxBuf[];               //串口2存放真实数据的数组（不包含包头包尾）
extern uint8_t UART2_RxFlag;                //串口2接收完成标志位（接收完成则为1）
extern uint8_t UART3_RxNewData;             //串口3最新接收到的数据
extern uint8_t UART3_RxBuf[];               //串口3存放真实数据的数组（不包含包头包尾）
extern uint8_t UART3_RxFlag;                //串口3接收完成标志位（接收完成则为1）
extern uint8_t UART5_RxNewData;             //串口5最新接收到的数据
extern uint8_t UART5_RxBuf[];               //串口5存放真实数据的数组（不包含包头包尾）
extern __IO uint8_t UART5_RxFlag;                //串口5接收完成标志位（接收完成则为1）


void UART1_Printf(char *fmt, ...);
void UART2_Printf(char *fmt, ...);
void UART3_Printf(char *fmt, ...);
void UART4_Printf(char *fmt, ...);


/******************当串口1开启DMA接收时启用下面的宏定义********************/
#if UART1_USE_DMA
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern uint8_t UART1_RxRealLength;              //串口1每次接收指令的实际长度
#endif
/***********************************************************************/

/***************当串口2接到测距模块GY53时启用下面的宏定义*******************/
#if UART2_USE_GY53
#define GY53_RxNewData          UART2_RxNewData
#define GY53_RxBuf              UART2_RxBuf
#define GY53_RxFlag             UART2_RxFlag
#define GY53_RxLength           UART2_RxLength
#define GY53_TxLengthMax        UART2_TxLengthMax
#define GY53_Printf             UART2_Printf
#define GY53_RxState            UART2_RxState
#define GY53_RxBufIdxNow        UART2_RxBufIdxNow
extern uint16_t GY53_Distance;                  //GY53模块返回的测量距离，单位：mm
#endif
/***********************************************************************/

/*****************当串口3接到蓝牙模块时启用下面的宏定义********************/
#if UART3_USE_BlueTeeth
#define BT_RxNewData        UART3_RxNewData
#define BT_RxBuf            UART3_RxBuf
#define BT_RxFlag           UART3_RxFlag
#define BT_RxLength         UART3_RxLength
#define BT_TxLengthMax      UART3_TxLengthMax
#define BT_Printf           UART3_Printf
#endif
/***********************************************************************/

/*****************当串口5接到步进电机时启用下面的宏定义********************/
#if UART5_USE_SteppingMotor
#define STEP_RxNewData        UART5_RxNewData
#define STEP_RxBuf            UART5_RxBuf
#define STEP_RxFlag           UART5_RxFlag
#define STEP_RxLength         UART5_RxLength
#define STEP_TxLengthMax      UART5_TxLengthMax
#define STEP_Printf           UART5_Printf
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart5_tx;
extern uint8_t STEP_RxRealLength;              //步进电机每次接收指令的实际长度
#endif
/***********************************************************************/


#endif
