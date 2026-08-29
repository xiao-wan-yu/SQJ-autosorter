#ifndef __CIRCLE_H
#define __CIRCLE_H

#include "circle_params.h"   /* ★ 所有可调参数集中在此，含每个参数"调大/调小"详细注释 */
#include "pid.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ============================================================================
   定半径圆周运动模块（三层闭环，按豆包方案实现）

   场景：车头正前方安装测距传感器（GY53，距车中心 CIRCLE_SENSOR_OFFSET_CM），
   绕外径半径 CIRCLE_PIPE_RADIUS_CM 的水管做定半径圆周运动。目标测距
   d_target_mm（传感器→管壁）在 150~200mm 内都要能实现，越圆越好。

   控制架构（从外到内，麦轮整车速度三层解耦）：
     目标半径 D_target = d_target + 传感器偏置 + 管半径
       ┌─[径向距离环 PID] → v_y（车头方向/径向：太远前进、太近后退，保半径）
     目标角速度 ω  ──[切向速度前馈] → v_x = ω·D_target（车身横向/切向）
       └─[航向同步环 PID] → Δω； w = ω 前馈 + Δω（车头始终指向圆心）
   辅助机制：
     1. 测距一阶低通滤波（抗噪声，避免距离环高频抖）
     2. 无效测距保护（测空/杂散时保持上次滤波值，距离环不动、只走切向+前馈）
     3. 极值搜索法修正陀螺仪漂移（正对管壁时测距最小，爬山微调航向目标角，
        200ms 一小步，带宽远低于角度/距离环，互不干扰）

   坐标系（与 chassis.c 完全一致）：
     v_x 车身右移为正、v_y 车头前进为正、w 逆时针为正(rad/s)
     HWT101CT yaw：0~360°、顺时针为正；逆时针绕圈时 yaw 递减
   ============================================================================ */

/* 可在线调参数结构体（默认值/含义/调大调小效果：见 circle_params.h 集中参数表）
   串口在线调参名见 serialplot.c param 表 */
typedef struct{
  /* 调试阶段：0 纯开环 → 1 加测距 → 2 加距离环 → 3 加航向环 → 4 加漂移修正（串口 cstage 切换） */
  int      stage;
  /* 径向距离环 PID */
  float    kp;                  // 距离环比例（串口 ckp）
  float    ki;                  // 距离环积分（串口 cki）
  float    kd;                  // 距离环微分（串口 ckd）
  float    int_max;             // 距离环积分限幅
  float    vy_max;              // 径向速度限幅 cm/s（串口 cvy）
  /* 航向同步环 */
  float    yaw_kp;              // 航向环比例，负值（串口 cyawkp）
  float    w_max;               // w 总输出限幅 rad/s（串口 cw）
  /* 测距滤波与有效性 */
  float    alpha;               // 低通滤波系数 0~1（串口 calpha）
  uint16_t d_min;               // 测距有效下限 mm（杂散丢弃）
  uint16_t d_max;               // 测距有效上限 mm（测空丢弃）
  /* 极值搜索漂移修正 */
  uint8_t  drift_enable;        // 1 开启 / 0 关闭
  float    drift_step;          // 漂移修正步长 °（串口 cstep）
  int      drift_period_ms;     // 漂移修正周期 ms（串口 cdriftper）
  /* 调试 */
  int      print_period_ms;     // 串口打印周期 ms，0 关闭（串口 cprint）
} CIRCLE_Param;

/* 运行状态结构体 */
typedef struct{
  uint8_t  running;             // 运行中
  int8_t   dir;                 // 方向：+1 逆时针 / -1 顺时针
  float    d_filt;              // 滤波后测距 cm
  float    D_actual;            // 实际轨迹半径 cm（车中心→圆心，实测）
  float    D_target;            // 目标轨迹半径 cm
  float    omega;               // 公转角速度 rad/s
  float    alpha;               // 公转累计角 rad
  float    yaw_start;           // 起始 yaw °
  float    yaw_offset;          // 漂移修正量 °
  float    yaw_acc;             // 陀螺仪累积转角 °（判断绕行弧角）
  float    yaw_last;            // 上次 yaw °
  PID_POS  dist_pid;            // 径向距离环（out → v_y）
  PID_POS  yaw_pid;             // 航向同步环（out → Δω）
  uint32_t tick_last;           // 上次控制时刻
  uint32_t drift_last;          // 上次漂移修正时刻
  float    drift_ref_d;         // 漂移修正参考测距 cm
  int8_t   drift_dir;           // 当前漂移步长方向（+1/-1）
  uint32_t print_last;          // 上次打印时刻
} CIRCLE_State;

extern CIRCLE_Param circle_param;
extern CIRCLE_State  circle;

/**
  * @brief 定半径圆周运动（阻塞式，绕满弧角自动停）
  * @param dist_gpio   测距传感器 GPIO 端口（如 GY53_2_GPIO_Port）
  * @param dist_pin    测距传感器 GPIO 引脚（如 GY53_2_Pin）
  * @param d_target_mm 目标测距 mm（传感器→管壁，150~200 典型，对应轨迹半径 27~32cm）
  * @param omega       公转角速度 rad/s（>0；切向速度 v_x = omega·D_target 自动算）
  * @param arc_deg     绕行弧角 °（360 = 整圈；到角自动停）
  * @param dir         方向：1 逆时针 / -1 顺时针
  * @note  前置条件：flag.chassis=1 且 flag.hwt101ct=1（陀螺仪数据在更新）。
  *        绕圈期间 flag.angle 临时置 0（w 由本模块接管），结束恢复并重新锁向。
  *        调用前请确保车头已正对水管（CIRCLE_Run 内部会先采样测距定初始半径）。
  */
void CIRCLE_Run(GPIO_TypeDef *dist_gpio, uint16_t dist_pin,
                uint16_t d_target_mm, float omega, uint32_t arc_deg, int8_t dir);

/**
  * @brief 单步控制（供 CIRCLE_Run 循环调用；也暴露给需要非阻塞集成的场景）
  * @param dist_gpio 测距 GPIO 端口
  * @param dist_pin  测距 GPIO 引脚
  * @retval 0 继续 / 1 已绕满弧角（调用方应停止并收尾）
  * @note  内部按 HAL_GetTick 差值计算 dt，自动控制节拍（无需外部延时）；
  *        GY53 读取为阻塞式（最大几十 ms），本函数本身不可重入。
  */
uint8_t CIRCLE_Step(GPIO_TypeDef *dist_gpio, uint16_t dist_pin);

/**
  * @brief 串口打印一次当前状态（用于 SerialPlot 观察收敛），单行多通道空格分隔
  */
void CIRCLE_PrintState(void);

#endif /* __CIRCLE_H */
