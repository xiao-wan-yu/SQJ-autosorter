/**
  * @file    tcs34725.h
  * @brief   TCS34725 RGBC 颜色传感器驱动（软件 I2C）接口
  * @note    本驱动用于替换原"1 号灰度传感器"（GRAY1）做颜色识别：
  *             SCL = PB9（原 GRAY1_CLK 位置）
  *             SDA = PB4（原 GRAY1_DATA 位置）
  *             VCC = 3.3V，GND = GND，LED / INT 悬空
  *          软件 I2C 100kHz（开漏输出 + 内部上拉），不依赖硬件 I2C 外设，
  *          因此不需要改动 CubeMX 工程。GRAY3 循线功能不受影响。
  *          如需更换引脚，只需改本文件末尾的 TCS34725_XXX_Pin/Port 四个宏。
  */

#ifndef __TCS34725_H
#define __TCS34725_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* ==================== I2C 地址（TCS34725 固定 0x29，无地址引脚） ==================== */
#define TCS34725_ADDRESS        (0x29u)
#define TCS34725_COMMAND_BIT    (0x80u)   /* 访问寄存器时命令字节必须置位 */

/* ==================== 寄存器 ==================== */
#define TCS34725_ENABLE         (0x00u)
#define TCS34725_ENABLE_AIEN    (0x10u)   /* RGBC 中断使能 */
#define TCS34725_ENABLE_WEN     (0x08u)   /* Wait 定时器使能 */
#define TCS34725_ENABLE_AEN     (0x02u)   /* RGBC ADC 使能 */
#define TCS34725_ENABLE_PON     (0x01u)   /* 内部振荡器上电 */
#define TCS34725_ATIME          (0x01u)   /* 积分时间 */
#define TCS34725_WTIME          (0x03u)   /* Wait 时间 */
#define TCS34725_AILTL          (0x04u)   /* Clear 通道中断低阈值（低字节） */
#define TCS34725_AILTH          (0x05u)   /* Clear 通道中断低阈值（高字节） */
#define TCS34725_AIHTL          (0x06u)   /* Clear 通道中断高阈值（低字节） */
#define TCS34725_AIHTH          (0x07u)   /* Clear 通道中断高阈值（高字节） */
#define TCS34725_PERS           (0x0Cu)   /* 中断持续滤波寄存器 */
#define TCS34725_CONFIG         (0x0Du)   /* 配置寄存器（WLONG 位） */
#define TCS34725_CONTROL        (0x0Fu)   /* 增益寄存器 */
#define TCS34725_ID             (0x12u)   /* ID：0x44=TCS34721/25，0x4D=TCS34723/27 */
#define TCS34725_STATUS         (0x13u)   /* 状态寄存器 */
#define TCS34725_STATUS_AINT    (0x10u)   /* 中断标志 */
#define TCS34725_STATUS_AVALID  (0x01u)   /* RGBC 完成一次积分，数据有效 */
#define TCS34725_CDATAL         (0x14u)   /* Clear 通道数据（低字节） */
#define TCS34725_CDATAH         (0x15u)   /* Clear 通道数据（高字节） */
#define TCS34725_RDATAL         (0x16u)   /* Red 通道数据（低字节） */
#define TCS34725_RDATAH         (0x17u)   /* Red 通道数据（高字节） */
#define TCS34725_GDATAL         (0x18u)   /* Green 通道数据（低字节） */
#define TCS34725_GDATAH         (0x19u)   /* Green 通道数据（高字节） */
#define TCS34725_BDATAL         (0x1Au)   /* Blue 通道数据（低字节） */
#define TCS34725_BDATAH         (0x1Bu)   /* Blue 通道数据（高字节） */

/* ==================== 积分时间 ==================== */
#define TCS34725_INTEGRATIONTIME_2_4MS   (0xFFu)   /*  2.4ms，计数上限 1024 */
#define TCS34725_INTEGRATIONTIME_24MS    (0xF6u)   /*   24ms，计数上限 10240 */
#define TCS34725_INTEGRATIONTIME_50MS    (0xEBu)   /*   50ms，计数上限 20480（默认） */
#define TCS34725_INTEGRATIONTIME_101MS   (0xD5u)   /*  101ms，计数上限 43008 */
#define TCS34725_INTEGRATIONTIME_154MS   (0xC0u)   /*  154ms，计数上限 65535 */
#define TCS34725_INTEGRATIONTIME_700MS   (0x00u)   /*  700ms，计数上限 65535 */

/* ==================== 增益 ==================== */
#define TCS34725_GAIN_1X        (0x00u)
#define TCS34725_GAIN_4X        (0x01u)
#define TCS34725_GAIN_16X       (0x02u)
#define TCS34725_GAIN_60X       (0x03u)

/* ==================== 颜色分类结果 ==================== */
#define TCS_COLOR_UNKNOWN   0
#define TCS_COLOR_BLACK     1
#define TCS_COLOR_WHITE     2
#define TCS_COLOR_RED       3
#define TCS_COLOR_ORANGE    4
#define TCS_COLOR_YELLOW    5
#define TCS_COLOR_GREEN     6
#define TCS_COLOR_CYAN      7
#define TCS_COLOR_BLUE      8
#define TCS_COLOR_MAGENTA   9

/* ==================== 颜色分类阈值（按实际物体/环境光标定） ====================
 * 判色采用"色相优先"（见 TCS34725_ClassifyColor）：
 *   蓝相区 H∈[200,280) -> 蓝；红相区 H<20 或 H>=340 -> 红；
 *   低饱和 s<TCS_WHITE_S_THRESH -> 按 v 分黑/白；
 *   其余按 h 区间扩展 橙/黄/绿/青/品红。
 * 2026-08-30 实测：黑 S=0.33 偏高、蓝 S=0.12 偏低、黑 V(0.43) > 蓝 V(0.38)，
 *   所以 TCS_WHITE_S_THRESH 须调到 0.35 左右才能把"黑"收进黑白分支。
 * 白 V 待测：TCS_WHITE_V_THRESH 最终取 (黑V + 白V)/2。 */
#define TCS_BLACK_V_THRESH   0.06f   /* 备用（色相优先方案当前未直接使用） */
#define TCS_WHITE_S_THRESH   0.35f   /* 饱和度低于此判灰/白（黑 S=0.33 需略高于 0.33） */
#define TCS_WHITE_V_THRESH   0.55f   /* 黑白分界：取黑V 与 白V 的中间值，白测后微调 */

/* ==================== 一次采样的完整结果 ==================== */
typedef struct {
  uint16_t c;    /* Clear（无滤色）原始 16 位值 */
  uint16_t r;    /* Red 原始 16 位值 */
  uint16_t g;    /* Green 原始 16 位值 */
  uint16_t b;    /* Blue 原始 16 位值 */
  float rn;      /* 归一化 rn = R/C（0~1，消除亮度影响） */
  float gn;      /* 归一化 gn = G/C */
  float bn;      /* 归一化 bn = B/C */
  float h;       /* HSV 色相 0~360 */
  float s;       /* HSV 饱和度 0~1 */
  float v;       /* HSV 亮度 0~1 */
  float lux;     /* 环境照度近似值（相对值） */
} TCS34725_RGBC;

/* ==================== 函数接口 ==================== */
uint8_t       TCS34725_Init(void);                 // 初始化 I2C + 读 ID + 设 50ms/1x + 使能，返回 1=在线
uint8_t       TCS34725_GetID(void);                // 返回读到的芯片 ID（0x44/0x4D），未在线返回 0
uint8_t       TCS34725_GetRawData(TCS34725_RGBC *rgbc); // 读 C/R/G/B 原始值并算归一化/HSV/lux，返回 1=成功
void          TCS34725_SetIntegrationTime(uint8_t it);  // 设积分时间（用上面 TCS34725_INTEGRATIONTIME_xxx）
void          TCS34725_SetGain(uint8_t gain);           // 设增益（用上面 TCS34725_GAIN_xxx）
void          TCS34725_Enable(void);                    // PON + AEN 上电并使能 ADC
void          TCS34725_Disable(void);                   // 关闭 ADC + 断电
uint8_t       TCS34725_ClassifyColor(const TCS34725_RGBC *rgbc); // 按 HSV 判颜色，返回 TCS_COLOR_xxx
const char   *TCS34725_ColorName(uint8_t color);        // 颜色结果转英文名，供 OLED/串口显示

#endif /* __TCS34725_H */
