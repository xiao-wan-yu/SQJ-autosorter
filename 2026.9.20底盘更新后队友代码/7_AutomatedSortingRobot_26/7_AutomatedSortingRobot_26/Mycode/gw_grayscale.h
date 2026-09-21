#ifndef __GW_GRAYSCALE_H
#define __GW_GRAYSCALE_H

/**
  * @brief 感为灰度传感器（串行 IO 接口）驱动
  * @note  两个灰度（GRAY1/GRAY3）都走感为官方例程的"IO 模式"（GPIO 模拟时钟），
  *        与 I2C 总线完全隔离。引脚由 CubeMX 配置（见 main.h / gpio.c）：
  *          GRAY1：DAT=PB4（输入上拉）、CLK=PB9（推挽输出）
  *          GRAY3：DAT=PB6（输入上拉）、CLK=PB7（推挽输出）
  *         CLK 下降沿采样 DAT 一位，循环 8 次读回 8 路数字量（0=深、1=浅）
  */

#include <stdint.h>

/* ==================== 灰度编号（作 GRAY_Data[] 索引，第0号废弃） ==================== */
#define GRAY1  1
#define GRAY2  2
#define GRAY3  3

extern uint8_t GRAY_Data[4][8];   // GRAY_Data[GRAYx][0~7] 对应探头1~8，串行读回的数字量：0=深（黑）、1=浅（白）
                                  // 深/浅判断由调用方（队友任务代码）自己写，例：GRAY_Data[GRAYx][i] == 0 → 深

void GRAY_Update(void);           // 一次刷新：GRAY1、GRAY3 各串行读 8 路数字量进 GRAY_Data
void GRAY1_Serial_Update(void);   // 单独刷新 GRAY1（串行接口）
void GRAY3_Serial_Update(void);   // 单独刷新 GRAY3（串行接口）

/* ============================================================================
 *  I2C 模拟量（颜色识别）接口 —— 感为官方协议，软件 I2C 复用各模块自己的 CLK/DAT
 *  ============================================================================
 *  原理：本模块（带MCU版）内部已把 8 路红外反射强度 ADC 成 0~255，
 *        通过 I2C 从寄存器读出（CLK=SCL、DAT=SDA，模块自动识别 I2C 时序）。
 *        数值越大 = 反射越强 = 表面越浅（白≈255、黑≈个位数）。
 *  接线：不需要改线。GRAY1 走 PB9(SCL)/PB4(SDA)，GRAY3 走 PB7(SCL)/PB6(SDA)，
 *        每路独立总线 => 两个模块地址相同（0x4F）也不冲突。
 *  注意：模块进入 I2C 模式后，该模块就不再输出串行数字量了；
 *        数字量改用寄存器 0xDD 读，见 GRAY_GetDigitalI2C()。
 * ========================================================================== */

/* 感为模块 I2C 寄存器（移植自官方例程 HardWare_IIC/hardware_iic.c） */
#define GW_GRAY_ADDR_DEF      0x4F   /* 7位从机地址候选1：AD1/AD0 均接高 */
#define GW_GRAY_ADDR_ALT      0x4C   /* 7位从机地址候选2：AD1/AD0 全低（出厂默认） */
#define GW_GRAY_PING_REG      0xAA   /* 在线检测寄存器，读到 0x66 表示在线 */
#define GW_GRAY_PING_OK       0x66
#define GW_GRAY_DIGITAL_REG   0xDD   /* 8路数字量，bit0~bit7 = 探头1~8（I2C 方式读数字量） */
#define GW_GRAY_ANALOG_BASE   0xB0   /* 模拟量基地址：从 0xB0 起连续读 8 字节 = 探头1~8，每路 0~255 */
#define GW_GRAY_ANALOG_CH(n)  (GW_GRAY_ANALOG_BASE + (n))   /* 单路模拟量，n=1~8 */
#define GW_GRAY_CH_ENABLE_REG 0xCE   /* 模拟量通道使能，写 0xFF 使能全部 8 路 */
#define GW_GRAY_NORMALIZE_REG 0xCF   /* 归一化：写 0xFF 全通道归一化、0x00 关闭（v3.6及以后固件） */

/* 颜色分类结果 */
#define GRAY_COLOR_BLACK   0
#define GRAY_COLOR_RED     1
#define GRAY_COLOR_BLUE    2
#define GRAY_COLOR_WHITE   3
#define GRAY_COLOR_UNKNOWN 4

/* 颜色阈值（必须按实际物体实测标定！）：
 *   value <= th_black_blue   => 黑
 *   value <= th_blue_red     => 蓝
 *   value <= th_red_white    => 红
 *   value >  th_red_white    => 白
 *   即认为黑 < 蓝 < 红 < 白（红外反射强度从低到高）。
 *   若实测红/蓝次序相反，把两个阈值对调即可。 */
typedef struct {
  uint16_t th_black_blue;    /* 黑/蓝分界 */
  uint16_t th_blue_red;      /* 蓝/红分界 */
  uint16_t th_red_white;     /* 红/白分界 */
} GRAY_ColorThresholds;

uint8_t  GRAY_I2C_Init(uint8_t module);                       // 软I2C初始化+自动探测地址+在线检测，返回1=在线
uint8_t  GRAY_ScanAddrs(uint8_t module, uint8_t *hit);        // 扫描4个候选地址应答情况(诊断用)，返回应答数
uint8_t  GRAY_ReadReg(uint8_t module, uint8_t reg, uint8_t *val);  // 读单个 I2C 寄存器(诊断用)，1=成功
uint8_t  GRAY_ReadSDA(uint8_t module);         // 读 SDA(DAT)空闲电平：1=高(未拉低) 0=低(被模块拉低)
uint8_t  GRAY_PingAny(uint8_t module, uint8_t addr); // 对任意7位地址做Ping，1=收到ACK（诊断用）
uint8_t  GRAY_GetAnalogAll(uint8_t module, uint8_t buf[8]);   // 连续读 8 路模拟量(0~255)，1=成功（内部自动重试初始化）
uint8_t  GRAY_GetSingleAnalog(uint8_t module, uint8_t ch);    // 读单路模拟量 ch=1~8，返回 0~255
uint8_t  GRAY_GetDigitalI2C(uint8_t module);                  // I2C 方式读 8 路数字量(bit0~7=探头1~8)
uint8_t  GRAY_GetDevAddr(uint8_t module);                     // 返回探测到的 7 位从机地址（0x4F/0x4C），0=未在线
uint8_t  GRAY_ColorClassify(uint16_t value, const GRAY_ColorThresholds *th);  // 按阈值判色
const char *GRAY_ColorName(uint8_t color);                    // 颜色结果转英文名，供 OLED/串口显示

#endif
