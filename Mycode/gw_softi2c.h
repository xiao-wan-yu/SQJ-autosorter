#ifndef __GW_SOFTI2C_H
#define __GW_SOFTI2C_H

/**
  * @brief 通用软件 I2C（位带模拟）主机驱动
  * @note  SCL/SDA 使用任意 GPIO（开漏输出 + 内部上拉），支持从机时钟拉伸。
  *        适用于感为灰度传感器等 I2C 从机——每个模块单独占一根 2 线总线，
  *        可避免多个同地址模块挂同一总线造成的地址冲突。
  *        时序约 100kHz（半位延时 5us），见 gw_softi2c.c 中 GW_I2C_DELAY_US。
  */

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* 一个软件 I2C 总线的引脚描述 */
typedef struct {
  GPIO_TypeDef *scl_port;
  uint16_t      scl_pin;
  GPIO_TypeDef *sda_port;
  uint16_t      sda_pin;
} GW_SoftI2C_t;

/* 把 SCL/SDA 两脚配成开漏输出 + 内部上拉，并置总线空闲（两根线都为高） */
void    GW_SoftI2C_Init(const GW_SoftI2C_t *bus);

/* 带寄存器地址的读/写（等价 HAL_I2C_Mem_Read / Mem_Write），返回 1=成功 0=失败 */
uint8_t GW_SoftI2C_Mem_Read (const GW_SoftI2C_t *bus, uint8_t dev_addr, uint8_t reg,
                             uint8_t *dst, uint8_t len);
uint8_t GW_SoftI2C_Mem_Write(const GW_SoftI2C_t *bus, uint8_t dev_addr, uint8_t reg,
                             const uint8_t *src, uint8_t len);

/* 仅发送从机地址探测设备是否在线，返回 1=在线 0=无应答 */
uint8_t GW_SoftI2C_Ping(const GW_SoftI2C_t *bus, uint8_t dev_addr);

#endif /* __GW_SOFTI2C_H */
