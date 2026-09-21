#ifndef __LOBOT_SERVO_H
#define __LOBOT_SERVO_H

/**
  * @brief 乐幻索尔 LSC 系列舵机控制板驱动（移植自去年 F103 工程 User/drivers/LobotServoController.c/h）
  * @note  硬件连接：UART5 —— PC12(UART5_TX) / PD2(UART5_RX)，波特率 9600（见 usart.c / .ioc）
  *         （CN6 舵机控制器接口，见《26年中国机器人大赛自动分拣机器人主控板》原理图）
  *         协议：帧头 0x55 0x55 + 数据长度 + 指令字 + 数据，详见《LSC系列舵机控制板二次开发手册》
  *         动作组号、舵机号/位置需先用乐幻索尔上位机(舵机调试软件)烧录/配置到控制板内
  */

#include <stdint.h>

/* ==================== LSC 控制板协议常量 ==================== */
#define FRAME_HEADER            0x55   // 帧头
#define CMD_SERVO_MOVE          0x03   // 舵机移动指令
#define CMD_ACTION_GROUP_RUN    0x06   // 运行动作组指令
#define CMD_ACTION_GROUP_STOP   0x07   // 停止运行动作组指令
#define CMD_ACTION_GROUP_SPEED  0x0B   // 设置动作组运行速度指令
#define CMD_GET_BATTERY_VOLTAGE 0x0F   // 获取电池电压指令

/* ==================== 舵机ID + 目标位置 结构体 ==================== */
typedef struct _lobot_servo_ {
    uint8_t  ID;       // 舵机ID（0~31）
    uint16_t Position; // 目标位置（0~1000 对应 0°~270°）
} LobotServo;

extern uint8_t  LobotTxBuf[128];  // 发送缓存
extern uint8_t  LobotRxBuf[16];   // 接收缓存（控制板回传数据，如需读取电压等可配合串口接收使用）
extern uint16_t batteryVolt;      // 电池电压，单位 0.1V（调用 getBatteryVoltage() 后需在接收中解析）

/* ==================== 舵机控制接口 ==================== */
void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time);          // 控制单个舵机转动
void moveServosByArray(LobotServo servos[], uint8_t Num, uint16_t Time);    // 数组方式控制多个舵机同时转动
void moveServos(uint8_t Num, uint16_t Time, ...);                           // 变参方式控制多个舵机：moveServos(Num, Time, ID1,Pos1, ID2,Pos2, ...)

/* ==================== 动作组控制接口 ====================
 * 注：动作组调用逻辑（如"先回初始位、再抓取、再放置"等）由用户自行编写，
 *     本文件只提供底层协议驱动
 * ======================================================= */
void runActionGroup(uint8_t numOfAction, uint16_t Times);   // 运行动作组，Times=0 时无限循环
void stopActionGroup(void);                                 // 停止所有动作组
void setActionGroupSpeed(uint8_t numOfAction, uint16_t Speed); // 设置指定动作组速度(百分比)
void setAllActionGroupSpeed(uint16_t Speed);                // 设置所有动作组速度(百分比)

/* ==================== 其他 ==================== */
void getBatteryVoltage(void);   // 发送获取电池电压命令（回传数据需配置串口接收后解析）
void receiveHandle(void);       // 接收数据处理入口（当前为占位，需按二次开发手册扩展）

#endif
