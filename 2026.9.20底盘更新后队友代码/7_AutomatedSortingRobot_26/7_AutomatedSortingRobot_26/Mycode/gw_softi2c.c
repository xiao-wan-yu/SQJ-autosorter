/**
  * @brief 通用软件 I2C（位带模拟）主机驱动实现
  * @note  引脚必须配成开漏输出：HAL_GPIO_WritePin(..., SET) = 释放线靠上拉拉高，
  *        RESET = 主动拉低。读 SDA 时先 SET 释放再读 IDR。
  *        支持从机时钟拉伸（SCL 拉高后轮询直到真正为高，带超时防死等）。
  */

#include "gw_softi2c.h"
#include "delay.h"

#define GW_I2C_DELAY_US  50u   /* 半位周期延时，约 10kHz（与官方硬件 I2C 例程的 ClockSpeed=10000 一致，
                                  模块内部是 MCU 软件模拟 I2C 从机，太快会丢应答） */

/* ---------------- 引脚电平操作 ---------------- */
static inline void scl_hi(const GW_SoftI2C_t *b){ HAL_GPIO_WritePin(b->scl_port, b->scl_pin, GPIO_PIN_SET); }
static inline void scl_lo(const GW_SoftI2C_t *b){ HAL_GPIO_WritePin(b->scl_port, b->scl_pin, GPIO_PIN_RESET); }
static inline void sda_hi(const GW_SoftI2C_t *b){ HAL_GPIO_WritePin(b->sda_port, b->sda_pin, GPIO_PIN_SET); }
static inline void sda_lo(const GW_SoftI2C_t *b){ HAL_GPIO_WritePin(b->sda_port, b->sda_pin, GPIO_PIN_RESET); }
static inline uint8_t sda_rd(const GW_SoftI2C_t *b){ return HAL_GPIO_ReadPin(b->sda_port, b->sda_pin) == GPIO_PIN_SET; }

/* SCL 拉高并等待从机释放（时钟拉伸）；返回 0=超时 */
static uint8_t scl_hi_wait(const GW_SoftI2C_t *b)
{
  uint32_t t = 0;
  HAL_GPIO_WritePin(b->scl_port, b->scl_pin, GPIO_PIN_SET);
  while(HAL_GPIO_ReadPin(b->scl_port, b->scl_pin) == GPIO_PIN_RESET){
    if(++t > 2000000u) return 0;
  }
  return 1;
}

/* ---------------- 时序 ---------------- */
static void i2c_start(const GW_SoftI2C_t *b)
{
  sda_hi(b);
  scl_hi(b);
  delay_us(GW_I2C_DELAY_US);
  sda_lo(b);                     /* SDA 在 SCL 高电平期间拉低 => 起始 */
  delay_us(GW_I2C_DELAY_US);
  scl_lo(b);
  delay_us(GW_I2C_DELAY_US);
}

static void i2c_stop(const GW_SoftI2C_t *b)
{
  scl_lo(b);
  sda_lo(b);
  delay_us(GW_I2C_DELAY_US);
  scl_hi(b);                     /* SDA 在 SCL 高电平期间拉高 => 停止 */
  delay_us(GW_I2C_DELAY_US);
  sda_hi(b);
  delay_us(GW_I2C_DELAY_US);
}

/* 写一个字节，返回 1=从机应答 */
static uint8_t i2c_write_byte(const GW_SoftI2C_t *b, uint8_t data)
{
  uint8_t i;
  for(i = 0; i < 8; i++){
    if(data & 0x80) sda_hi(b); else sda_lo(b);     /* MSB 先发 */
    delay_us(GW_I2C_DELAY_US);
    if(!scl_hi_wait(b)) return 0;
    delay_us(GW_I2C_DELAY_US);
    scl_lo(b);
    delay_us(GW_I2C_DELAY_US);
    data <<= 1;
  }
  /* 第 9 个时钟：读 ACK（从机拉低 SDA = 应答） */
  sda_hi(b);
  delay_us(GW_I2C_DELAY_US);
  if(!scl_hi_wait(b)) return 0;
  uint8_t ack = (sda_rd(b) == 0) ? 1 : 0;
  delay_us(GW_I2C_DELAY_US);
  scl_lo(b);
  delay_us(GW_I2C_DELAY_US);
  return ack;
}

/* 读一个字节；ack=0 主机发 ACK，ack=1 主机发 NACK（读最后一字节用 NACK） */
static uint8_t i2c_read_byte(const GW_SoftI2C_t *b, uint8_t ack)
{
  uint8_t data = 0, i;
  sda_hi(b);                     /* 主机释放 SDA，交给从机驱动 */
  for(i = 0; i < 8; i++){
    delay_us(GW_I2C_DELAY_US);
    if(!scl_hi_wait(b)) return 0xFF;
    data = (uint8_t)((data << 1) | (sda_rd(b) ? 1u : 0u));
    delay_us(GW_I2C_DELAY_US);
    scl_lo(b);
    delay_us(GW_I2C_DELAY_US);
  }
  /* 第 9 个时钟：主机发 ACK(0)/NACK(1) */
  if(ack) sda_hi(b); else sda_lo(b);
  delay_us(GW_I2C_DELAY_US);
  if(!scl_hi_wait(b)) return 0xFF;
  delay_us(GW_I2C_DELAY_US);
  scl_lo(b);
  delay_us(GW_I2C_DELAY_US);
  sda_hi(b);
  return data;
}

/* ---------------- 公共接口 ---------------- */

void GW_SoftI2C_Init(const GW_SoftI2C_t *bus)
{
  GPIO_InitTypeDef g = {0};
  g.Mode  = GPIO_MODE_OUTPUT_OD;   /* 开漏输出：高=释放线，低=拉低 */
  g.Pull  = GPIO_PULLUP;           /* 模块板载若无上拉，用内部上拉保底 */
  g.Speed = GPIO_SPEED_FREQ_HIGH;

  g.Pin = bus->scl_pin;
  HAL_GPIO_Init(bus->scl_port, &g);
  g.Pin = bus->sda_pin;
  HAL_GPIO_Init(bus->sda_port, &g);

  sda_hi(bus);
  scl_hi(bus);                     /* 总线空闲 */
}

uint8_t GW_SoftI2C_Mem_Read(const GW_SoftI2C_t *bus, uint8_t dev_addr, uint8_t reg,
                            uint8_t *dst, uint8_t len)
{
  uint8_t i;
  i2c_start(bus);
  if(!i2c_write_byte(bus, (uint8_t)(dev_addr << 1)))      { i2c_stop(bus); return 0; }  /* 写地址 */
  if(!i2c_write_byte(bus, reg))                           { i2c_stop(bus); return 0; }  /* 寄存器地址 */
  i2c_start(bus);                                          /* 重复起始 */
  if(!i2c_write_byte(bus, (uint8_t)((dev_addr << 1) | 1)) ){ i2c_stop(bus); return 0; } /* 读地址 */
  for(i = 0; i < len; i++){
    dst[i] = i2c_read_byte(bus, (i < len - 1) ? 0 : 1);   /* 前 n-1 字节 ACK，最后 1 字节 NACK */
  }
  i2c_stop(bus);
  return 1;
}

uint8_t GW_SoftI2C_Mem_Write(const GW_SoftI2C_t *bus, uint8_t dev_addr, uint8_t reg,
                             const uint8_t *src, uint8_t len)
{
  uint8_t i;
  i2c_start(bus);
  if(!i2c_write_byte(bus, (uint8_t)(dev_addr << 1))) { i2c_stop(bus); return 0; }
  if(!i2c_write_byte(bus, reg))                       { i2c_stop(bus); return 0; }
  for(i = 0; i < len; i++){
    if(!i2c_write_byte(bus, src[i]))                  { i2c_stop(bus); return 0; }
  }
  i2c_stop(bus);
  return 1;
}

uint8_t GW_SoftI2C_Ping(const GW_SoftI2C_t *bus, uint8_t dev_addr)
{
  uint8_t ok;
  i2c_start(bus);
  ok = i2c_write_byte(bus, (uint8_t)(dev_addr << 1));
  i2c_stop(bus);
  return ok;
}
