/**
  * @brief 乐幻索尔 LSC 系列舵机控制板驱动（移植自去年 F103 工程 User/drivers/src/LobotServoController.c）
  * @note  通信串口：UART5（PC12_UART5_TX / PD2_UART5_RX，CN6 舵机控制器接口），波特率 9600
  *         原 F103 工程用标准外设库 Usart_SendArray() 发送，本工程已改为 HAL 库 HAL_UART_Transmit()
  * @usage 示例：
  *           moveServo(1, 2000, 1000);            // 1号舵机 1000ms 转到位置2000
  *           moveServos(2, 800, 2,1200, 9,2300);  // 2号舵机1200、9号舵机2300，800ms 同时到达
  *           runActionGroup(1, 1);                // 1号动作组运行1次（动作组逻辑由用户自行编写）
  *           stopActionGroup();                   // 停止动作组
  * @warning 舵机ID范围 0~31；Position 范围 0~1000（0°~270°）；Time 单位 ms
  */

#include "lobot_servo.h"
#include "stm32f4xx_hal.h"   // HAL_UART_Transmit / UART_HandleTypeDef
#include <stdarg.h>

extern UART_HandleTypeDef huart5;   // 舵机控制板所在串口（见 Core/Src/usart.c）

/* ==================== 底层宏定义 ==================== */
#define GET_LOW_BYTE(A)  ((uint8_t)(A))          // 取低八位
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))   // 取高八位

#define LSC_UART_TIMEOUT  1000   // 串口发送超时时间(ms)

/* ==================== 全局缓存 ==================== */
uint8_t  LobotTxBuf[128];  // 发送缓存
uint8_t  LobotRxBuf[16];   // 接收缓存
uint16_t batteryVolt;      // 电池电压(0.1V)

/**
  * @brief 底层串口发送：通过 UART5 阻塞发送一帧数据
  * @param buf 待发送数据
  * @param len 数据长度
  */
static void LSC_SendArray(uint8_t *buf, uint16_t len)
{
    HAL_UART_Transmit(&huart5, buf, len, LSC_UART_TIMEOUT);
}

/*********************************************************************************
 * Function:  moveServo
 * Description： 控制单个舵机转动
 * Parameters:   servoID:舵机ID，Position:目标位置, Time:转动时间
 *               舵机ID取值:0<=舵机ID<=31, Time取值: Time > 0
 * Return:       无返回
 **********************************************************************************/
void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time)
{
    if (servoID > 31 || !(Time > 0)) {  // 舵机ID不能大于31,可根据对应控制板修改
        return;
    }
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;   // 填充帧头
    LobotTxBuf[2] = 8;                              // 数据长度=要控制舵机数*3+5，此处=1*3+5
    LobotTxBuf[3] = CMD_SERVO_MOVE;                 // 填充舵机移动指令
    LobotTxBuf[4] = 1;                              // 要控制的舵机个数
    LobotTxBuf[5] = GET_LOW_BYTE(Time);             // 取得时间的低八位
    LobotTxBuf[6] = GET_HIGH_BYTE(Time);            // 取得时间的高八位
    LobotTxBuf[7] = servoID;                        // 舵机ID
    LobotTxBuf[8] = GET_LOW_BYTE(Position);         // 取得目标位置的低八位
    LobotTxBuf[9] = GET_HIGH_BYTE(Position);        // 取得目标位置的高八位

    LSC_SendArray(LobotTxBuf, 10);
}

/*********************************************************************************
 * Function:  moveServosByArray
 * Description： 控制多个舵机同时转动（数组方式）
 * Parameters:   servos[]:舵机结构体数组，Num:舵机个数, Time:转动时间
 *               0 < Num <= 32, Time > 0
 * Return:       无返回
 **********************************************************************************/
void moveServosByArray(LobotServo servos[], uint8_t Num, uint16_t Time)
{
    uint8_t index = 7;
    uint8_t i = 0;

    if (Num < 1 || Num > 32 || !(Time > 0)) {
        return;  // 舵机数不能为零和大与32，时间不能为零
    }
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;      // 填充帧头
    LobotTxBuf[2] = Num * 3 + 5;                       // 数据长度 = 要控制舵机数*3+5
    LobotTxBuf[3] = CMD_SERVO_MOVE;                    // 填充舵机移动指令
    LobotTxBuf[4] = Num;                               // 要控制的舵机个数
    LobotTxBuf[5] = GET_LOW_BYTE(Time);                // 取得时间的低八位
    LobotTxBuf[6] = GET_HIGH_BYTE(Time);               // 取得时间的高八位

    for (i = 0; i < Num; i++) {                        // 循环填充舵机ID和对应目标位置
        LobotTxBuf[index++] = servos[i].ID;            // 填充舵机ID
        LobotTxBuf[index++] = GET_LOW_BYTE(servos[i].Position);  // 填充目标位置低八位
        LobotTxBuf[index++] = GET_HIGH_BYTE(servos[i].Position); // 填充目标位置高八位
    }

    LSC_SendArray(LobotTxBuf, LobotTxBuf[2] + 2);
}

/*********************************************************************************
 * Function:  moveServos
 * Description： 控制多个舵机同时转动（可变参数方式）
 * Parameters:   Num:舵机个数, Time:转动时间, ...:舵机ID,转动角度，舵机ID,转动角度 如此类推
 * Return:       无返回
 * 示例: moveServos(2, 800, 2, 1200, 9, 2300);  // 2个舵机,800ms,2号舵机转1200,9号舵机转2300
 **********************************************************************************/
void moveServos(uint8_t Num, uint16_t Time, ...)
{
    uint8_t index = 7;
    uint8_t i = 0;
    uint16_t temp;
    va_list arg_ptr;

    va_start(arg_ptr, Time);   // 取得可变参数首地址
    if (Num < 1 || Num > 32) {
        va_end(arg_ptr);
        return;                // 舵机数不能为零和大与32
    }
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;      // 填充帧头
    LobotTxBuf[2] = Num * 3 + 5;                // 数据长度 = 要控制舵机数 * 3 + 5
    LobotTxBuf[3] = CMD_SERVO_MOVE;             // 舵机移动指令
    LobotTxBuf[4] = Num;                        // 要控制舵机数
    LobotTxBuf[5] = GET_LOW_BYTE(Time);         // 取得时间的低八位
    LobotTxBuf[6] = GET_HIGH_BYTE(Time);        // 取得时间的高八位

    for (i = 0; i < Num; i++) {                 // 从可变参数中取得并循环填充舵机ID和对应目标位置
        temp = va_arg(arg_ptr, int);            // 取得舵机ID
        LobotTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp));
        temp = va_arg(arg_ptr, int);            // 取得对应目标位置
        LobotTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp));  // 填充目标位置低八位
        LobotTxBuf[index++] = GET_HIGH_BYTE(temp);            // 填充目标位置高八位
    }

    va_end(arg_ptr);

    LSC_SendArray(LobotTxBuf, LobotTxBuf[2] + 2);
}

/*********************************************************************************
 * Function:  runActionGroup
 * Description： 运行指定动作组
 * Parameters:   numOfAction:动作组序号, Times:执行次数
 * Return:       无返回
 * Others:       Times = 0 时无限循环
 **********************************************************************************/
void runActionGroup(uint8_t numOfAction, uint16_t Times)
{
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;   // 填充帧头
    LobotTxBuf[2] = 5;                      // 数据长度，此命令固定为5
    LobotTxBuf[3] = CMD_ACTION_GROUP_RUN;   // 填充运行动作组命令
    LobotTxBuf[4] = numOfAction;            // 填充要运行的动作组号
    LobotTxBuf[5] = GET_LOW_BYTE(Times);    // 取得要运行次数的低八位
    LobotTxBuf[6] = GET_HIGH_BYTE(Times);   // 取得要运行次数的高八位

    LSC_SendArray(LobotTxBuf, 7);
}

/*********************************************************************************
 * Function:  stopActionGroup
 * Description： 停止动作组运行
 * Parameters:   无
 * Return:       无返回
 **********************************************************************************/
void stopActionGroup(void)
{
    LobotTxBuf[0] = FRAME_HEADER;             // 填充帧头
    LobotTxBuf[1] = FRAME_HEADER;
    LobotTxBuf[2] = 2;                        // 数据长度，此命令固定为2
    LobotTxBuf[3] = CMD_ACTION_GROUP_STOP;    // 填充停止运行动作组命令

    LSC_SendArray(LobotTxBuf, 4);
}

/*********************************************************************************
 * Function:  setActionGroupSpeed
 * Description： 设定指定动作组的运行速度
 * Parameters:   numOfAction: 动作组序号, Speed:目标速度(百分比)
 * Return:       无返回
 **********************************************************************************/
void setActionGroupSpeed(uint8_t numOfAction, uint16_t Speed)
{
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;   // 填充帧头
    LobotTxBuf[2] = 5;                       // 数据长度，此命令固定为5
    LobotTxBuf[3] = CMD_ACTION_GROUP_SPEED;  // 填充设置动作组速度命令
    LobotTxBuf[4] = numOfAction;             // 填充要设置的动作组号
    LobotTxBuf[5] = GET_LOW_BYTE(Speed);     // 获得目标速度的低八位
    LobotTxBuf[6] = GET_HIGH_BYTE(Speed);    // 获得目标速度的高八位

    LSC_SendArray(LobotTxBuf, 7);
}

/*********************************************************************************
 * Function:  setAllActionGroupSpeed
 * Description： 设置所有动作组的运行速度
 * Parameters:   Speed: 目标速度(百分比)
 * Return:       无返回
 **********************************************************************************/
void setAllActionGroupSpeed(uint16_t Speed)
{
    setActionGroupSpeed(0xFF, Speed);  // 组号为0xFF时设置所有组的速度
}

/*********************************************************************************
 * Function:  getBatteryVoltage
 * Description： 发送获取电池电压命令
 * Parameters:   无
 * Return:       无返回
 * Others:       控制板回传电压数据，需开启 UART5 接收(中断/DMA)并填充 receiveHandle() 解析
 **********************************************************************************/
void getBatteryVoltage(void)
{
    LobotTxBuf[0] = FRAME_HEADER;            // 填充帧头
    LobotTxBuf[1] = FRAME_HEADER;
    LobotTxBuf[2] = 2;                       // 数据长度，此命令固定为2
    LobotTxBuf[3] = CMD_GET_BATTERY_VOLTAGE; // 填充获取电池电压命令

    LSC_SendArray(LobotTxBuf, 4);
}

/*********************************************************************************
 * Function:  receiveHandle
 * Description： 舵机控制板回传数据处理入口（当前为占位）
 * Others:       如需解析电池电压等回传数据：
 *               1. 配置 UART5 接收中断/DMA，把收到的一帧存入 LobotRxBuf；
 *               2. 在此函数中按二次开发手册解析，例如：
 *                  switch (LobotRxBuf[3]) {
 *                  case CMD_GET_BATTERY_VOLTAGE:
 *                      batteryVolt = (uint16_t)((LobotRxBuf[5] << 8) | LobotRxBuf[4]);
 *                      break;
 *                  default: break;
 *                  }
 **********************************************************************************/
void receiveHandle(void)
{
    /* 可根据二次开发手册添加其他指令解析 */
}

