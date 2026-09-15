/**
  * @file    tcs34725.h
  * @brief   TCS34725 RGBC 颜色传感器驱动（软件 I2C）接口
  * @note    本驱动用于替换原"1 号灰度传感器"（GRAY1）做颜色识别：
  *             SCL = PB9（原 GRAY1_CLK 位置）
  *             SDA = PB4（原 GRAY1_DATA 位置）
  *             VCC = 3.3V，GND = GND，LED / INT 悬空
  *          软件 I2C 100kHz（开漏输出 + 内部上拉），不依赖硬件 I2C 外设，
  *          因此不需要改动 CubeMX 工程。GRAY3 循线功能不受影响。
  *          如需更换引脚，只需改 tcs34725.c 开头的 TCS34725_XXX_Pin/Port 四个宏。
  *
  *          ★使用前必须先初始化一次：TCS34725_Init()（把 PB9/PB4 配成开漏 + 上电使能）。
  *            正常情况下由 main.c 上电处调用；万一调用方漏了，TCS34725_GetRawData()
  *            也会自己补一次（见 tcs34725.c 的 tcs_ensure_ready），不会再"悄悄坏掉"。
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
#define TCS_COLOR_NONBLACK  10
#define TCS_COLOR_WHITE     2
#define TCS_COLOR_RED       3
#define TCS_COLOR_ORANGE    4
#define TCS_COLOR_YELLOW    5
#define TCS_COLOR_GREEN     6
#define TCS_COLOR_CYAN      7
#define TCS_COLOR_BLUE      8
#define TCS_COLOR_MAGENTA   9

/* ==================== 颜色分类阈值（★2026-09-15 按板上 CAL 模式实测重定：LED 亮、增益1x、50ms） ====================
 * 判色分 黑 / 红 / 蓝 三色，用 S、V 分类（不用 H —— 实测蓝的 H 不在蓝相区）：
 *   2026-09-15 实测（每种颜色 K2 采 5 次取平均；S/V 为各次范围 + 均值）：
 *     黑 S=0.300~0.375(均0.349) V=0.571~0.647(均0.604) C=13~17
 *     红 S=0.464~0.500(均0.479) V=0.758~0.800(均0.779) C=31~40
 *     蓝 S=0.125~0.133(均0.130) V=0.577~0.600(均0.588) C=25~27
 *     白 S=0.213~0.234(均0.225) V=0.423~0.443(均0.436) C=98~107
 *   排序： S 蓝 < 白 < 黑 < 红（互不重叠，这才是能分开四色的关键）；V 白 < 蓝≈黑 < 红。
 * 规则（见 TCS34725_ClassifyColor）：
 *   低饱和 s<TCS_BLUE_S_MAX -> V<TCS_BLUE_V_MAX 判蓝；否则判黑（白/灰，不参与比赛）
 *   其余有彩色          -> V<TCS_BLACK_V_THRESH 判黑；否则判红
 * 三个阈值 = "相邻两类实测值的中点"：
 *   TCS_BLUE_S_MAX     = (蓝S最大0.133 + 白S最小0.213)/2 ≈ 0.17
 *   TCS_BLUE_V_MAX     =  蓝V最大0.600 + 余量            ≈ 0.65
 *   TCS_BLACK_V_THRESH = (黑V最大0.647 + 红V最小0.758)/2  ≈ 0.70
 * 微调口诀：蓝被误判黑 → 调大 TCS_BLUE_V_MAX；黑被判红 → 调大 TCS_BLACK_V_THRESH；
 *           蓝被判黑(却 S 偏大) → 调大 TCS_BLUE_S_MAX；白/黑被判蓝 → 看它 S 是否掉到阈值下。
 * ★前提：照明要稳定（模块 LED 脚接 3.3V 常亮；接 GND = 关断不亮）。换光/换距离就重采重定。 */
#define TCS_BLUE_S_MAX     0.17f   /* 低饱和阈值（蓝 S≤0.133，白 S≥0.213，取中间） */
#define TCS_BLUE_V_MAX     0.65f   /* 低饱和时亮度低于此判蓝（蓝 V≤0.600） */
#define TCS_BLACK_V_THRESH 0.70f   /* 有彩色时亮度低于此判黑（黑 V≤0.647，红 V≥0.758） */
#define TCS_WHITE_S_THRESH 0.35f   /* 备用 */
#define TCS_WHITE_V_THRESH 0.55f   /* 备用 */

/* ★阈值怎么重定：开机进 KEY0 起点里的 CAL 校准模式 → 黑/红/蓝/白 各按 K2 采 5 次
 *   （串口打印 C/R/G/B 原始值 + H/S/V + 当前判色结果），拿四色实测均值按"相邻两类取中间值"
 *   改下面 3 个宏，重新编译烧录即可（阈值就是这个编译期宏，程序运行时不改它）。 */

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
