/**
  * @file    tcs34725.c
  * @brief   TCS34725 RGBC 颜色传感器驱动实现（软件 I2C 位带模拟）
  * @note    替换原"1 号灰度传感器"（GRAY1）：
  *          硬件上把 TCS34725 模块接到原 GRAY1 的线位上——
  *            SCL = PB9（原 GRAY1_CLK）、SDA = PB4（原 GRAY1_DATA）
  *            VCC = 3.3V、GND = GND、LED/INT 悬空
  *          软件 I2C 100kHz（半位延时 5us，开漏输出 + 内部上拉，支持时钟拉伸）。
  *          与感为灰度驱动（gw_softi2c，10kHz）相互独立，互不影响。
  */

#include "tcs34725.h"
#include "delay.h"          // delay_us / delay_ms（DWT 延时库）

/* ==================== 引脚定义（改这里即可换引脚） ==================== */
#define TCS34725_SCL_GPIO_Port  GPIOB
#define TCS34725_SCL_Pin        GPIO_PIN_9
#define TCS34725_SDA_GPIO_Port  GPIOB
#define TCS34725_SDA_Pin        GPIO_PIN_4

#define TCS_I2C_HALF_DELAY_US   5u    /* 半位周期延时 5us => 约 100kHz */

static uint8_t g_tcs_id = 0;         /* 上电后读到的芯片 ID */

/* ==================== 引脚电平操作 ==================== */
static inline void tcs_scl_hi(void){ HAL_GPIO_WritePin(TCS34725_SCL_GPIO_Port, TCS34725_SCL_Pin, GPIO_PIN_SET); }
static inline void tcs_scl_lo(void){ HAL_GPIO_WritePin(TCS34725_SCL_GPIO_Port, TCS34725_SCL_Pin, GPIO_PIN_RESET); }
static inline void tcs_sda_hi(void){ HAL_GPIO_WritePin(TCS34725_SDA_GPIO_Port, TCS34725_SDA_Pin, GPIO_PIN_SET); }
static inline void tcs_sda_lo(void){ HAL_GPIO_WritePin(TCS34725_SDA_GPIO_Port, TCS34725_SDA_Pin, GPIO_PIN_RESET); }
static inline uint8_t tcs_sda_rd(void){ return (HAL_GPIO_ReadPin(TCS34725_SDA_GPIO_Port, TCS34725_SDA_Pin) == GPIO_PIN_SET) ? 1u : 0u; }

/* SCL 拉高并等待从机释放（时钟拉伸，带超时防死等） */
static uint8_t tcs_scl_hi_wait(void)
{
  uint32_t t = 0;
  HAL_GPIO_WritePin(TCS34725_SCL_GPIO_Port, TCS34725_SCL_Pin, GPIO_PIN_SET);
  while(HAL_GPIO_ReadPin(TCS34725_SCL_GPIO_Port, TCS34725_SCL_Pin) == GPIO_PIN_RESET){
    if(++t > 2000000u) return 0;
  }
  return 1;
}

/* ==================== I2C 时序（主机模式） ==================== */
static void tcs_i2c_start(void)
{
  tcs_sda_hi(); tcs_scl_hi();
  delay_us(TCS_I2C_HALF_DELAY_US);
  tcs_sda_lo();                       /* SDA 在 SCL 高电平期间拉低 => 起始 */
  delay_us(TCS_I2C_HALF_DELAY_US);
  tcs_scl_lo();
  delay_us(TCS_I2C_HALF_DELAY_US);
}

static void tcs_i2c_stop(void)
{
  tcs_scl_lo(); tcs_sda_lo();
  delay_us(TCS_I2C_HALF_DELAY_US);
  tcs_scl_hi();                       /* SDA 在 SCL 高电平期间拉高 => 停止 */
  delay_us(TCS_I2C_HALF_DELAY_US);
  tcs_sda_hi();
  delay_us(TCS_I2C_HALF_DELAY_US);
}

/* 写一个字节，返回 1=从机 ACK，0=NACK/超时 */
static uint8_t tcs_i2c_write_byte(uint8_t data)
{
  uint8_t i;
  for(i = 0; i < 8; i++){
    if(data & 0x80) tcs_sda_hi(); else tcs_sda_lo();   /* MSB 先发 */
    delay_us(TCS_I2C_HALF_DELAY_US);
    if(!tcs_scl_hi_wait()) return 0;
    delay_us(TCS_I2C_HALF_DELAY_US);
    tcs_scl_lo();
    delay_us(TCS_I2C_HALF_DELAY_US);
    data <<= 1;
  }
  /* 第 9 个时钟：读 ACK（从机拉低 SDA = 应答） */
  tcs_sda_hi();
  delay_us(TCS_I2C_HALF_DELAY_US);
  if(!tcs_scl_hi_wait()) return 0;
  uint8_t ack = (tcs_sda_rd() == 0) ? 1u : 0u;
  delay_us(TCS_I2C_HALF_DELAY_US);
  tcs_scl_lo();
  delay_us(TCS_I2C_HALF_DELAY_US);
  return ack;
}

/* 读一个字节；ack=0 主机发 ACK，ack=1 主机发 NACK（读最后一字节用 NACK） */
static uint8_t tcs_i2c_read_byte(uint8_t ack)
{
  uint8_t data = 0, i;
  tcs_sda_hi();                       /* 主机释放 SDA，交给从机驱动 */
  for(i = 0; i < 8; i++){
    delay_us(TCS_I2C_HALF_DELAY_US);
    if(!tcs_scl_hi_wait()) return 0xFF;
    data = (uint8_t)((data << 1) | (tcs_sda_rd() ? 1u : 0u));
    delay_us(TCS_I2C_HALF_DELAY_US);
    tcs_scl_lo();
    delay_us(TCS_I2C_HALF_DELAY_US);
  }
  /* 第 9 个时钟：主机发 ACK(0)/NACK(1) */
  if(ack) tcs_sda_hi(); else tcs_sda_lo();
  delay_us(TCS_I2C_HALF_DELAY_US);
  if(!tcs_scl_hi_wait()) return 0xFF;
  delay_us(TCS_I2C_HALF_DELAY_US);
  tcs_scl_lo();
  delay_us(TCS_I2C_HALF_DELAY_US);
  tcs_sda_hi();
  return data;
}

/* 把 SCL/SDA 配成开漏输出 + 内部上拉并置总线空闲 */
static void tcs_i2c_gpio_init(void)
{
  GPIO_InitTypeDef g = {0};
  g.Mode  = GPIO_MODE_OUTPUT_OD;      /* 开漏：高=释放线，低=拉低 */
  g.Pull  = GPIO_PULLUP;              /* 模块板载一般有 4.7k~10k 上拉，内部上拉保底 */
  g.Speed = GPIO_SPEED_FREQ_HIGH;

  g.Pin = TCS34725_SCL_Pin;
  HAL_GPIO_Init(TCS34725_SCL_GPIO_Port, &g);
  g.Pin = TCS34725_SDA_Pin;
  HAL_GPIO_Init(TCS34725_SDA_GPIO_Port, &g);

  tcs_sda_hi();
  tcs_scl_hi();                       /* 总线空闲 */
}

/* ==================== 寄存器读写（命令位 0x80 自动加上） ==================== */
static uint8_t tcs_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
  uint8_t i;
  tcs_i2c_start();
  if(!tcs_i2c_write_byte((uint8_t)((TCS34725_ADDRESS << 1) | 0x00))) { tcs_i2c_stop(); return 0; }
  if(!tcs_i2c_write_byte((uint8_t)(TCS34725_COMMAND_BIT | reg)))     { tcs_i2c_stop(); return 0; }
  tcs_i2c_start();                                 /* 重复起始 */
  if(!tcs_i2c_write_byte((uint8_t)((TCS34725_ADDRESS << 1) | 0x01))) { tcs_i2c_stop(); return 0; }
  for(i = 0; i < len; i++){
    buf[i] = tcs_i2c_read_byte((i < len - 1) ? 0u : 1u);   /* 前 n-1 字节 ACK，最后 1 字节 NACK */
  }
  tcs_i2c_stop();
  return 1;
}

static uint8_t tcs_write_reg(uint8_t reg, uint8_t val)
{
  tcs_i2c_start();
  if(!tcs_i2c_write_byte((uint8_t)((TCS34725_ADDRESS << 1) | 0x00))) { tcs_i2c_stop(); return 0; }
  if(!tcs_i2c_write_byte((uint8_t)(TCS34725_COMMAND_BIT | reg)))     { tcs_i2c_stop(); return 0; }
  if(!tcs_i2c_write_byte(val))                                       { tcs_i2c_stop(); return 0; }
  tcs_i2c_stop();
  return 1;
}

static uint16_t tcs_read16(uint8_t reg)
{
  uint8_t d[2] = {0, 0};
  if(tcs_read_regs(reg, d, 2)) return (uint16_t)(((uint16_t)d[1] << 8) | d[0]);
  return 0;
}

/* ==================== RGB(归一化分量 0~1) -> HSV ==================== */
static void rgb_to_hsv(float r, float g, float b, float *h, float *s, float *v)
{
  float max = r, min = r;
  if(g > max) max = g;
  if(g < min) min = g;
  if(b > max) max = b;
  if(b < min) min = b;
  float d = max - min;

  *v = max;
  if(max < 0.0001f || d < 0.0001f){ *h = 0.0f; *s = 0.0f; return; }

  *s = d / max;
  if(max == r)      *h = 60.0f * ((g - b) / d);
  else if(max == g) *h = 60.0f * ((b - r) / d + 2.0f);
  else              *h = 60.0f * ((r - g) / d + 4.0f);

  if(*h < 0.0f)   *h += 360.0f;
  if(*h >= 360.0f) *h -= 360.0f;
}

/* ==================== 公共接口 ==================== */

void TCS34725_SetIntegrationTime(uint8_t it)
{
  tcs_write_reg(TCS34725_ATIME, it);
}

void TCS34725_SetGain(uint8_t gain)
{
  /* CONTROL 寄存器 bit0~1 是增益，bit2~7 保留置 0 */
  tcs_write_reg(TCS34725_CONTROL, (uint8_t)(gain & 0x03u));
}

void TCS34725_Enable(void)
{
  uint8_t e = TCS34725_ENABLE_PON;
  tcs_write_reg(TCS34725_ENABLE, e);
  delay_ms(5);                                   /* PON 后需 >=2.4ms 等待内部振荡器稳定 */
  e = TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN;
  tcs_write_reg(TCS34725_ENABLE, e);
  delay_ms(60);                                  /* 等第一个积分周期完成（默认 50ms 积分时间） */
}

void TCS34725_Disable(void)
{
  uint8_t e = TCS34725_ENABLE_PON;
  tcs_read_regs(TCS34725_ENABLE, &e, 1);
  e = (uint8_t)(e & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));
  tcs_write_reg(TCS34725_ENABLE, e);
}

uint8_t TCS34725_GetID(void)
{
  return g_tcs_id;
}

uint8_t TCS34725_Init(void)
{
  uint8_t id = 0;

  tcs_i2c_gpio_init();
  delay_ms(5);                                   /* 让上拉稳定 */

  tcs_read_regs(TCS34725_ID, &id, 1);
  if(id != 0x44u && id != 0x4Du) return 0;       /* 0x44=TCS34721/25，0x4D=TCS34723/27 */
  g_tcs_id = id;

  TCS34725_SetIntegrationTime(TCS34725_INTEGRATIONTIME_50MS);
  TCS34725_SetGain(TCS34725_GAIN_1X);
  TCS34725_Enable();
  return 1;
}

uint8_t TCS34725_GetRawData(TCS34725_RGBC *rgbc)
{
  float c;
  if(!rgbc) return 0;

  /* 循环积分模式下数据持续更新，直接读最近一次积分结果；
     上电初期/刚 AEN 时数据可能为 0，用返回值 + C 值判断即可 */
  rgbc->c = tcs_read16(TCS34725_CDATAL);
  rgbc->r = tcs_read16(TCS34725_RDATAL);
  rgbc->g = tcs_read16(TCS34725_GDATAL);
  rgbc->b = tcs_read16(TCS34725_BDATAL);

  c = (float)rgbc->c;
  if(c > 0.001f){
    rgbc->rn = (float)rgbc->r / c;
    rgbc->gn = (float)rgbc->g / c;
    rgbc->bn = (float)rgbc->b / c;
  } else {
    rgbc->rn = 0.0f; rgbc->gn = 0.0f; rgbc->bn = 0.0f;
  }

  rgb_to_hsv(rgbc->rn, rgbc->gn, rgbc->bn, &rgbc->h, &rgbc->s, &rgbc->v);

  /* 照度近似（Adafruit 公式，输入为 16bit 原始值，输出为相对照度） */
  float lux = -0.32466f * (float)rgbc->r + 1.57837f * (float)rgbc->g - 0.73191f * (float)rgbc->b;
  rgbc->lux = (lux > 0.0f) ? lux : 0.0f;
  return 1;
}

uint8_t TCS34725_ClassifyColor(const TCS34725_RGBC *rgbc)
{
  if(!rgbc) return TCS_COLOR_UNKNOWN;

  /* ===== 三色：黑/红/蓝（2026-08-30 按 24 次采样数据标定）=====
     采样结果：
       黑 H≈20  S=0.43~0.50 V=0.44~0.50
       红 H≈0~4 S=0.70~0.74 V=0.66~0.71
       蓝 H≈300 S=0.10~0.14 V=0.36~0.38（蓝 S/V 都很低，H 不在蓝相区！）
     所以用 S、V 分类（不用 H）：
       低饱和 s<TCS_BLUE_S_MAX -> V<TCS_BLUE_V_MAX 判蓝；否则判黑（白/灰不参与比赛）
       其余有彩色          -> V<TCS_BLACK_V_THRESH 判黑；否则判红 */

  if(rgbc->s < TCS_BLUE_S_MAX){
    return (rgbc->v < TCS_BLUE_V_MAX) ? TCS_COLOR_BLUE : TCS_COLOR_BLACK;
  }
  if(rgbc->v < TCS_BLACK_V_THRESH) return TCS_COLOR_BLACK;
  return TCS_COLOR_RED;
}

const char *TCS34725_ColorName(uint8_t color)
{
  switch(color){
    case TCS_COLOR_BLACK:   return "BLACK";
    case TCS_COLOR_NONBLACK: return "NON-BLACK";
    case TCS_COLOR_WHITE:   return "WHITE";
    case TCS_COLOR_RED:     return "RED";
    case TCS_COLOR_ORANGE:  return "ORANGE";
    case TCS_COLOR_YELLOW:  return "YELLOW";
    case TCS_COLOR_GREEN:   return "GREEN";
    case TCS_COLOR_CYAN:    return "CYAN";
    case TCS_COLOR_BLUE:    return "BLUE";
    case TCS_COLOR_MAGENTA: return "MAGENTA";
    default:                return "????";
  }
}
