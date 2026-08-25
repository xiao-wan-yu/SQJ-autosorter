#ifndef __GW_GRAYSCALE_H
#define __GW_GRAYSCALE_H

/**
  * @brief 感为灰度传感器（硬件I2C）驱动
  * @note  从机地址 0x4C(7位)，调用 HAL 时需左移1位 -> 0x98
  *        硬件接口：I2C1 (PB6=SCL, PB7=SDA)，复用工程已初始化的 hi2c1
  *        移植自感为官方例程 Hal_Periphral/HardWare_IIC
  */

#include <stdint.h>

/* 从机地址（7位）：实测本模块 AD1/AD0 均接高，地址为 0x4F（默认 0x4C，仅当 AD1/AD0 全低时使用） */
#define GW_GRAY_ADDR_DEF 0x4F

/* 在线检测：读 0xAA 返回 0x66 表示在线 */
#define GW_GRAY_PING      0xAA
#define GW_GRAY_PING_OK   0x66
#define GW_GRAY_PING_RSP  GW_GRAY_PING_OK

/* 读 8 路数字量（开关量，0/1） */
#define GW_GRAY_DIGITAL_MODE 0xDD

/* 连续读模拟量基地址（0xB0 起，最多 8 路） */
#define GW_GRAY_ANALOG_BASE_ 0xB0
#define GW_GRAY_ANALOG_MODE  (GW_GRAY_ANALOG_BASE_ + 0)
/* 读单个探头模拟量，n 从 1 开始到 8 */
#define GW_GRAY_ANALOG(n)    (GW_GRAY_ANALOG_BASE_ + (n))

/* 传感器归一化寄存器（v3.6及之后固件） */
#define GW_GRAY_ANALOG_NORMALIZE 0xCF

/* 黑/白滞回比较参数操作 */
#define GW_GRAY_CALIBRATION_BLACK 0xD0
#define GW_GRAY_CALIBRATION_WHITE 0xD1

/* 设置所需探头的模拟信号(channel enable) */
#define GW_GRAY_ANALOG_CHANNEL_ENABLE 0xCE
#define GW_GRAY_ANALOG_CH_EN_1 (0x1 << 0)
#define GW_GRAY_ANALOG_CH_EN_2 (0x1 << 1)
#define GW_GRAY_ANALOG_CH_EN_3 (0x1 << 2)
#define GW_GRAY_ANALOG_CH_EN_4 (0x1 << 3)
#define GW_GRAY_ANALOG_CH_EN_5 (0x1 << 4)
#define GW_GRAY_ANALOG_CH_EN_6 (0x1 << 5)
#define GW_GRAY_ANALOG_CH_EN_7 (0x1 << 6)
#define GW_GRAY_ANALOG_CH_EN_8 (0x1 << 7)
#define GW_GRAY_ANALOG_CH_EN_ALL (0xFF)

/* 读取错误信息 */
#define GW_GRAY_ERROR 0xDE
/* 设备软件重启 */
#define GW_GRAY_REBOOT 0xC0
/* 读取固件版本号 */
#define GW_GRAY_FIRMWARE 0xC1
/* 设置设备I2C地址 */
#define GW_GRAY_CHANGE_ADDR 0xAD
/* 读偏移量（2字节小端） */
#define GW_GRAY_OFFSET 0x88

/* 广播重置地址所需要发的数据 */
#define GW_GRAY_BROADCAST_RESET "\xB8\xD0\xCE\xAA\xBF\xC6\xBC\xBC"

/**
  * @brief 从 8 位数字量中取第 n 位（n 从 1 开始，n=1 是第一个探头，n=8 是最后一个）
  */
#define GET_NTH_BIT(sensor_value, nth_bit) (((sensor_value) >> ((nth_bit)-1)) & 0x01)

/**
  * @brief 从一个变量分离出全部 8 个 bit
  */
#define SEP_ALL_BIT8(sensor_value, val1, val2, val3, val4, val5, val6, val7, val8) \
do {                                                                              \
val1 = GET_NTH_BIT(sensor_value, 1);                                              \
val2 = GET_NTH_BIT(sensor_value, 2);                                              \
val3 = GET_NTH_BIT(sensor_value, 3);                                              \
val4 = GET_NTH_BIT(sensor_value, 4);                                              \
val5 = GET_NTH_BIT(sensor_value, 5);                                              \
val6 = GET_NTH_BIT(sensor_value, 6);                                              \
val7 = GET_NTH_BIT(sensor_value, 7);                                              \
val8 = GET_NTH_BIT(sensor_value, 8);                                              \
} while(0)

/* ==================== 接口函数 ==================== */
uint8_t  GW_Gray_Init(void);                        // Ping 检测，0=在线，1=离线
uint8_t  GW_Gray_GetDigital(void);                  // 读 8 路数字量，bit0~bit7 对应探头1~8
uint8_t  GW_Gray_GetAnalog(uint8_t *buf, uint8_t len); // 连续读 len 路模拟量(从0xB0起)，1=成功
uint8_t  GW_Gray_GetSingleAnalog(uint8_t ch);       // 读单路模拟量，ch=1~8
uint8_t  GW_Gray_Normalize(uint8_t ch);             // 归一化开关，0xFF=开，0x00=关
uint16_t GW_Gray_GetOffset(void);                   // 读偏移量(2字节小端)

#endif
