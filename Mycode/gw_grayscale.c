/**
  * @brief 感为灰度传感器（串行 IO 接口）驱动实现
  * @note  两个灰度（GRAY1/GRAY3）都走感为官方例程的"IO 模式"（GPIO 模拟时钟）：
  *        CLK 下降沿采样 DAT，上升沿让模块刷新下一位，循环 8 次读回 8 路数字量（bit0~bit7 = 探头1~8）
  *        GPIO 由 CubeMX 配置（main.h/gpio.c）：GRAY1=PB4(DAT)/PB9(CLK)、GRAY3=PB6(DAT)/PB7(CLK)
  *        曾用 I2C1 读取（GRAY3 模拟量），因两个模块同挂 I2C 总线时 0x4C 读失败，
  *        CubeMX 已禁用 I2C1，全部改用串行接口
  */

#include "gw_grayscale.h"
#include "main.h"                    // GRAY1_DATA_Pin / GRAY1_CLK_Pin 等（CubeMX 生成）
#include "stm32f4xx_hal.h"           // HAL_GPIO
#include "delay.h"                   // delay_us：CLK 翻转微秒级延时

/* ==================== 灰度数据（GRAY1/GRAY3 走串行 IO 接口） ==================== */
uint8_t GRAY_Data[4][8] = {0};       // GRAY_Data[GRAYx][0~7] 对应探头1~8，0=深（黑）、1=浅（白）；第0号灰度废弃

/**
  * @brief  串行读 8 路数字量（IO 模式：CLK 下降沿采样 DAT，上升沿让模块刷新下一位）
  * @param  clk_port/clk_pin  CLK 引脚（主控输出，CubeMX 配成推挽输出，空闲拉低）
  * @param  dat_port/dat_pin  DAT 引脚（模块输出，CubeMX 配成输入上拉）
  * @retval 8bit，bit0~bit7 对应探头1~8；1=浅（白）、0=深（黑）
  * @note   时序移植自感为官方 Serial 例程：CLK 低延时 2us 读 DAT、拉高延时 5us
  */
static uint8_t GW_Serial_ReadBits(GPIO_TypeDef *clk_port, uint16_t clk_pin,
                                  GPIO_TypeDef *dat_port, uint16_t dat_pin)
{
  uint8_t ret = 0;
  for(uint8_t i = 0; i < 8; i++){
    HAL_GPIO_WritePin(clk_port, clk_pin, GPIO_PIN_RESET);            // 下降沿
    delay_us(2);
    ret |= (uint8_t)HAL_GPIO_ReadPin(dat_port, dat_pin) << i;         // 采样 DAT
    HAL_GPIO_WritePin(clk_port, clk_pin, GPIO_PIN_SET);              // 上升沿
    delay_us(5);
  }
  return ret;
}

/**
  * @brief  单独刷新 GRAY1：串行读 8 路数字量（0/1）存 GRAY_Data[GRAY1][0~7]
  * @note   若实测深浅极性相反（深显示成 1），把存储语句对调即可
  */
void GRAY1_Serial_Update(void)
{
  uint8_t bits = GW_Serial_ReadBits(GRAY1_CLK_GPIO_Port, GRAY1_CLK_Pin,
                                    GRAY1_DATA_GPIO_Port, GRAY1_DATA_Pin);
  for(uint8_t i = 0; i < 8; i++){
    GRAY_Data[GRAY1][i] = (bits >> i) & 0x01;
  }
}

/**
  * @brief  单独刷新 GRAY3：串行读 8 路数字量（0/1）存 GRAY_Data[GRAY3][0~7]
  */
void GRAY3_Serial_Update(void)
{
  uint8_t bits = GW_Serial_ReadBits(GRAY3_CLK_GPIO_Port, GRAY3_CLK_Pin,
                                    GRAY3_DATA_GPIO_Port, GRAY3_DATA_Pin);
  for(uint8_t i = 0; i < 8; i++){
    GRAY_Data[GRAY3][i] = (bits >> i) & 0x01;
  }
}

/**
  * @brief  一次刷新 GRAY_Data（深/浅判断由调用方做）
  * @note   GRAY1/GRAY3 都走串行接口读 8 路数字量（0=深、1=浅）
  *         GRAY2 暂不使用（启用时加一行 GRAY2_Serial_Update() 即可，需先配好引脚）
  */
void GRAY_Update(void)
{
  GRAY1_Serial_Update();
  /* GRAY2 暂不使用 */
  GRAY3_Serial_Update();
}
