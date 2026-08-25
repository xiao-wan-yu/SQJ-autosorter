/**
  * @brief 感为灰度传感器（硬件I2C）驱动实现
  * @note  移植自感为官方例程 HardWare_IIC/hardware_iic.c
  *        复用工程 I2C1 (hi2c1, PB6=SCL / PB7=SDA)，纯 HAL 阻塞式接口
  */

#include "gw_grayscale.h"
#include "i2c.h"     // 提供 extern hi2c1

/* 从机地址：HAL 的 DevAddress 需要 8 位左对齐地址（7位地址左移1位） */
#define GW_GRAY_DEV_ADDR  (GW_GRAY_ADDR_DEF << 1)

/**
  * @brief  I2C 读：带寄存器地址连续读（寄存器宽度 8bit，阻塞式）
  * @retval 1=成功，0=失败
  */
static uint8_t GW_IIC_ReadBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *dst, uint8_t len)
{
  return HAL_I2C_Mem_Read(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, dst, len, 1000) == HAL_OK;
}

/**
  * @brief  I2C 写：带寄存器地址连续写（寄存器宽度 8bit，阻塞式）
  * @retval 1=成功，0=失败
  */
static uint8_t GW_IIC_WriteBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *src, uint8_t len)
{
  return HAL_I2C_Mem_Write(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, src, len, 1000) == HAL_OK;
}

/**
  * @brief  在线检测（Ping）：读 0xAA 寄存器，返回 0x66 表示传感器在线
  * @retval 0=在线，1=离线
  */
uint8_t GW_Gray_Init(void)
{
  uint8_t dat = 0;
  GW_IIC_ReadBytes(GW_GRAY_DEV_ADDR, GW_GRAY_PING, &dat, 1);
  if (dat == GW_GRAY_PING_OK)
  {
    return 0;
  }
  return 1;
}

/**
  * @brief  读 8 路数字量（开关量）
  * @retval 8bit，bit0 对应探头1，bit7 对应探头8
  */
uint8_t GW_Gray_GetDigital(void)
{
  uint8_t dat = 0;
  GW_IIC_ReadBytes(GW_GRAY_DEV_ADDR, GW_GRAY_DIGITAL_MODE, &dat, 1);
  return dat;
}

/**
  * @brief  连续读多路模拟量（从 0xB0 起，len 个字节）
  * @param  buf  存放数据的数组
  * @param  len  读取路数（最多 8）
  * @retval 1=成功，0=失败
  */
uint8_t GW_Gray_GetAnalog(uint8_t *buf, uint8_t len)
{
  return GW_IIC_ReadBytes(GW_GRAY_DEV_ADDR, GW_GRAY_ANALOG_BASE_, buf, len);
}

/**
  * @brief  读单路模拟量
  * @param  ch  通道号，从 1 开始（1~8）
  * @retval 该通道灰度值（0~255）
  */
uint8_t GW_Gray_GetSingleAnalog(uint8_t ch)
{
  uint8_t dat = 0;
  GW_IIC_ReadBytes(GW_GRAY_DEV_ADDR, GW_GRAY_ANALOG(ch), &dat, 1);
  return dat;
}

/**
  * @brief  归一化开关
  * @param  ch  0xFF=打开全部通道归一化，0x00=关闭
  * @note   写入后需 HAL_Delay(10) 等待传感器刷新数据，再读取归一化值
  * @retval 1=成功，0=失败
  */
uint8_t GW_Gray_Normalize(uint8_t ch)
{
  return GW_IIC_WriteBytes(GW_GRAY_DEV_ADDR, GW_GRAY_ANALOG_NORMALIZE, &ch, 1);
}

/**
  * @brief  读偏移量（2 字节，小端序）
  * @retval 偏移量
  */
uint16_t GW_Gray_GetOffset(void)
{
  uint8_t dat[2] = {0};
  GW_IIC_ReadBytes(GW_GRAY_DEV_ADDR, GW_GRAY_OFFSET, dat, 2);
  return (uint16_t)dat[0] | (uint16_t)dat[1] << 8;
}
