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
  * @note   GRAY1 已被 TCS34725 颜色传感器替换（PB9/PB4 改作软件 I2C），
  *         不再走串行读取，见 Mycode/tcs34725.c。若恢复 GRAY1，取消下行注释即可。
  *         GRAY3 仍走串行接口读 8 路数字量（0=深、1=浅）
  *         GRAY2 暂不使用（启用时加一行 GRAY2_Serial_Update() 即可，需先配好引脚）
  */
void GRAY_Update(void)
{
  /* GRAY1 已替换为 TCS34725，禁止再驱动 PB9/PB4（会干扰 I2C） */
  /* GRAY1_Serial_Update(); */
  /* GRAY2 暂不使用 */
  GRAY3_Serial_Update();
}

/* ==================== I2C 模拟量（颜色识别）实现 ==================== */
/* 软件 I2C 复用各模块现有 CLK/DAT 引脚，每模块独立总线（避免同地址冲突）。
 * GRAY1: SCL=PB9(GRAY1_CLK)、SDA=PB4(GRAY1_DATA)
 * GRAY3: SCL=PB7(GRAY3_CLK)、SDA=PB6(GRAY3_DATA)
 * 注意：模块进入 I2C 模式后不再输出串行数字量，数字量改用 GRAY_GetDigitalI2C() 读 0xDD。 */
#include "gw_softi2c.h"

static const GW_SoftI2C_t gw_i2c_bus[4] = {
  {0, 0, 0, 0},                                                             /* 0 废弃 */
  {GRAY1_CLK_GPIO_Port, GRAY1_CLK_Pin, GRAY1_DATA_GPIO_Port, GRAY1_DATA_Pin}, /* GRAY1 */
  {0, 0, 0, 0},                                                             /* GRAY2 未接 */
  {GRAY3_CLK_GPIO_Port, GRAY3_CLK_Pin, GRAY3_DATA_GPIO_Port, GRAY3_DATA_Pin}, /* GRAY3 */
};

/* 每个模块探测到的 7 位从机地址，0=尚未成功 */
static uint8_t gw_dev_addr[4] = {0, 0, 0, 0};

/* 地址候选：感为模块 AD1/AD0 两位决定地址，4 种组合全试。
 *   00 -> 0x4C（出厂默认，官方 APP 用它）   01 -> 0x4D
 *   10 -> 0x4E                              11 -> 0x4F（旧板实测）
 * 模块 I2C 速度很低（约 10kHz），探测本身较慢，见 gw_softi2c.c 的 GW_I2C_DELAY_US。 */
static const uint8_t gw_addr_cand[4] = {GW_GRAY_ADDR_DEF, GW_GRAY_ADDR_ALT, 0x4D, 0x4E};

/**
  * @brief  软件 I2C 初始化某灰度模块：自动探测地址 + 在线检测
  * @param  module GRAY1 / GRAY3
  * @retval 1=在线，0=离线或参数错
  * @note   在线判定用“只发从机地址收到 ACK”（GW_SoftI2C_Ping），不依赖 0xAA 寄存器内容，
  *         兼容不同固件版本。上电后模块可能还没就绪，最多重试 5 次（间隔 100ms）。
  *         找到地址后顺便写 0xCE=0xFF 使能全部 8 路模拟量通道（即使默认已使能也无副作用）。
  *         成功后用 GRAY_GetDevAddr(module) 可查实际用到的地址。
  */
uint8_t GRAY_I2C_Init(uint8_t module)
{
  if(module < GRAY1 || module > GRAY3) return 0;
  GW_SoftI2C_Init(&gw_i2c_bus[module]);

  for(uint8_t retry = 0; retry < 5; retry++){
    for(uint8_t i = 0; i < 4; i++){
      uint8_t addr = gw_addr_cand[i];
      if(GW_SoftI2C_Ping(&gw_i2c_bus[module], addr)){
        uint8_t ch_en = 0xFF;
        GW_SoftI2C_Mem_Write(&gw_i2c_bus[module], addr, GW_GRAY_CH_ENABLE_REG, &ch_en, 1);
        gw_dev_addr[module] = addr;
        return 1;
      }
    }
    delay_ms(100);
  }
  gw_dev_addr[module] = 0;
  return 0;
}

/**
  * @brief  扫描 4 个候选地址的应答情况（诊断用），把有应答的地址写回 *hit
  * @param  module GRAY1 / GRAY3
  * @param  hit   若探测到有应答的地址则回填，否则置 0
  * @retval 有应答的地址数量
  * @note   只做 Ping，不改变任何状态；配合串口打印可确认模块到底用哪个地址。
  */
uint8_t GRAY_ScanAddrs(uint8_t module, uint8_t *hit)
{
  uint8_t n = 0;
  if(module < GRAY1 || module > GRAY3){ if(hit) *hit = 0; return 0; }
  GW_SoftI2C_Init(&gw_i2c_bus[module]);
  *hit = 0;
  for(uint8_t i = 0; i < 4; i++){
    if(GW_SoftI2C_Ping(&gw_i2c_bus[module], gw_addr_cand[i])){
      if(*hit == 0) *hit = gw_addr_cand[i];
      n++;
    }
  }
  return n;
}

/**
  * @brief  连续读 8 路模拟量（0~255，越大越浅）
  * @param  module GRAY1 / GRAY3
  * @param  buf    长度 8 的数组，buf[0~7] 对应探头 1~8
  * @retval 1=成功，0=失败
  * @note   若尚未初始化成功，内部会自动重试初始化，方便在主循环里直接调用。
  */
uint8_t GRAY_GetAnalogAll(uint8_t module, uint8_t buf[8])
{
  if(module < GRAY1 || module > GRAY3) return 0;
  if(gw_dev_addr[module] == 0 && !GRAY_I2C_Init(module)) return 0;
  return GW_SoftI2C_Mem_Read(&gw_i2c_bus[module], gw_dev_addr[module], GW_GRAY_ANALOG_BASE, buf, 8);
}

/**
  * @brief  读单路模拟量
  * @param  module GRAY1 / GRAY3
  * @param  ch     1~8
  * @retval 该通道模拟量 0~255；参数错或读失败返回 0
  */
uint8_t GRAY_GetSingleAnalog(uint8_t module, uint8_t ch)
{
  if(module < GRAY1 || module > GRAY3 || ch < 1 || ch > 8) return 0;
  if(gw_dev_addr[module] == 0 && !GRAY_I2C_Init(module)) return 0;
  uint8_t dat = 0;
  GW_SoftI2C_Mem_Read(&gw_i2c_bus[module], gw_dev_addr[module], GW_GRAY_ANALOG_CH(ch), &dat, 1);
  return dat;
}

/**
  * @brief  I2C 方式读 8 路数字量（模块进 I2C 模式后替代串行接口用）
  * @retval 8bit，bit0~bit7 = 探头 1~8，1=浅（白）、0=深（黑）；失败返回 0
  */
uint8_t GRAY_GetDigitalI2C(uint8_t module)
{
  if(module < GRAY1 || module > GRAY3) return 0;
  if(gw_dev_addr[module] == 0 && !GRAY_I2C_Init(module)) return 0;
  uint8_t dat = 0;
  GW_SoftI2C_Mem_Read(&gw_i2c_bus[module], gw_dev_addr[module], GW_GRAY_DIGITAL_REG, &dat, 1);
  return dat;
}

/**
  * @brief  返回某模块探测到的 7 位从机地址（0x4F 或 0x4C），未在线返回 0
  */
uint8_t GRAY_GetDevAddr(uint8_t module)
{
  if(module < GRAY1 || module > GRAY3) return 0;
  return gw_dev_addr[module];
}

/**
  * @brief  读单个 I2C 寄存器（诊断用：探测绿光款等不同固件的寄存器内容）
  * @retval 1=成功，0=失败（未在线/通讯错）
  */
uint8_t GRAY_ReadReg(uint8_t module, uint8_t reg, uint8_t *val)
{
  if(module < GRAY1 || module > GRAY3) return 0;
  if(gw_dev_addr[module] == 0 && !GRAY_I2C_Init(module)) return 0;
  return GW_SoftI2C_Mem_Read(&gw_i2c_bus[module], gw_dev_addr[module], reg, val, 1);
}

/**
  * @brief  读 SDA(DAT) 空闲电平（诊断用）
  * @retval 1=SDA 为高（模块没有拉低）；0=SDA 被拉低
  * @note   若释放 SDA 后仍为低，说明模块 DAT 一直被它自己驱动为低
  *         （可能并未进入 I2C 模式），此时地址"有 ACK"可能是假象。
  */
uint8_t GRAY_ReadSDA(uint8_t module)
{
  if(module < GRAY1 || module > GRAY3) return 0;
  GW_SoftI2C_Init(&gw_i2c_bus[module]);          /* 配成开漏并释放 SDA */
  delay_us(20);
  return (HAL_GPIO_ReadPin(gw_i2c_bus[module].sda_port, gw_i2c_bus[module].sda_pin) == GPIO_PIN_SET) ? 1 : 0;
}

/**
  * @brief  对任意 7 位地址做 Ping（诊断用）
  * @retval 1=该地址收到 ACK，0=无应答
  * @note   配合假地址（如 0x00/0x7F）一起测试：若假地址也"成功"，
  *         说明 SDA 被模块拉低导致 ACK 误判，模块并未真正支持 I2C。
  */
uint8_t GRAY_PingAny(uint8_t module, uint8_t addr)
{
  if(module < GRAY1 || module > GRAY3) return 0;
  GW_SoftI2C_Init(&gw_i2c_bus[module]);
  return GW_SoftI2C_Ping(&gw_i2c_bus[module], addr);
}

/**
  * @brief  按阈值把模拟量判为颜色
  * @note   前提：黑 < 蓝 < 红 < 白（红外反射强度从低到高），
  *         若实测红/蓝次序相反，把 th_blue_red 与 th_red_white 调换即可。
  */
uint8_t GRAY_ColorClassify(uint16_t value, const GRAY_ColorThresholds *th)
{
  if(value <= th->th_black_blue) return GRAY_COLOR_BLACK;
  if(value <= th->th_blue_red)   return GRAY_COLOR_BLUE;
  if(value <= th->th_red_white)  return GRAY_COLOR_RED;
  return GRAY_COLOR_WHITE;
}

const char *GRAY_ColorName(uint8_t color)
{
  switch(color){
    case GRAY_COLOR_BLACK: return "BLACK";
    case GRAY_COLOR_RED:   return "RED";
    case GRAY_COLOR_BLUE:  return "BLUE";
    case GRAY_COLOR_WHITE: return "WHITE";
    default:               return "????";
  }
}
