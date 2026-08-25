#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "pid.h"
#include "stm32f4xx_hal.h"
#include "velocityProfile.h"   // 梯形速度规划（go_to_xy 移植：mx/my 走固定距离）

/* ==================== 系统开关标志 ==================== */
/* 各模块由对应标志位开启/关闭（main.c 定义变量，chassis.c 等模块引用） */
typedef struct {
  uint8_t hwt101ct;   // 陀螺仪数据更新标志
  uint8_t chassis;    // 底盘控制循环标志
  uint8_t angle;      // 航向环（角度环）开关：1 开启（默认），0 关闭回手动 w
} FLAG;

extern FLAG flag;

/* 轮子编号：与 TB6612 电机编号、编码器编号、TIM1 PWM 通道一一对应 */
typedef enum {
    CHASSIS_MOTOR_LF = 1,  // 左前  TIM1_CH1
    CHASSIS_MOTOR_LB = 2,  // 左后  TIM1_CH2
    CHASSIS_MOTOR_RB = 3,  // 右后  TIM1_CH3
    CHASSIS_MOTOR_RF = 4,  // 右前  TIM1_CH4
} ChassisMotorIndex;

/* ==================== 车体可配置参数（待实测标定） ==================== */
#define WHEEL_DIAMETER   12.7f    // 实测：轮径 127mm
#define MECANUM_LENGTH   22.8f    // 实测：前后轮中心距离 cm（第二阶段用）
#define MECANUM_WIDTH    33.5f    // 实测：左右轮中心距离 cm（第二阶段用）
#define ENCODER_ACCURACY 439.0f   // 实测标定：手转轮10圈=8780脉冲 → 一圈878
#define ENCODER_TIME_S   0.010f   // 控制周期 10ms
#define PERIMETER        (3.14159265f * WHEEL_DIAMETER)  // 轮子周长 cm
/* ==================== 速度环 PID 参数（增量式，分段：不同目标速度区间用不同参数） ==================== */
#define SPEED_PID_KP       3.0f    // 仅作为未启用分段时的兜底值，分段模式下每周期按 target 选段覆盖
#define SPEED_PID_KI       0.05f
#define SPEED_PID_KD       0.0f
#define SPEED_PID_OUT_MAX  900.0f   // PWM输出对称限幅（已加大到900，实测最大车速~160cm/s）
#define SPEED_SEG_NUM      4        // 分段数
#define SPEED_SEG_BOUND_1  40.0f    // 段1上边界（0-40）
#define SPEED_SEG_BOUND_2  80.0f    // 段2上边界（40-80）
#define SPEED_SEG_BOUND_3  120.0f   // 段3上边界（80-120）
#define SPEED_TARGET_MAX   160.0f   // 4轮目标速度限幅（实测最大车速~160cm/s），防止麦轮解算出超限目标
/* ==================== 航向环（角度环）PID 参数 ==================== */
#define YAW_PID_KP       -0.028f  // 实测标定：kp 负（HWT101CT yaw 顺时针为正，与 w 定义镜像）
#define YAW_PID_KI       0.0f     // 实测：积分不需要（锁向纠偏靠 P 即可）
#define YAW_PID_KD       0.0f     // 实测：微分不需要（速度环自身已提供阻尼，加 D 反而抖）
#define YAW_PID_OUT_MAX  2.8f    // w 输出上限 rad/s
#define YAW_PID_OUT_MIN  (-2.8f) // w 输出下限 rad/s
#define YAW_DEAD_ZONE_TURN  1.0f    // 静止旋转死区（°）：误差进入该范围→w=0 + 4轮速度环清零停转
                                    // 防小w输出累加到增量式速度环PWM、克服静摩擦猛动造成来回飘动（实测有效）
#define YAW_DEAD_ZONE_MOVE  0.2f    // 走直线(有平移)死区（°）：更小，让1°内偏航也被角度环纠正，直线更直
                                    // 两套死区由控制循环按 v_x/v_y 是否≈0 自动切换，无需在线调
#define YAW_TARGET_NONE  (-999.0f)  // target_yaw 哨兵：首次进入控制循环时锁定当前陀螺仪朝向
/* ==================== 梯形速度规划（go_to_xy 移植：串口 mx/my 走固定距离自动停） ==================== */
#define MOVE_SPEED_DEFAULT  60.0f    // 规划目标速度 cm/s（serialplot param 表 mv 可在线调）
#define MOVE_ACC_DEFAULT    100.0f   // 规划加减速 cm/s²（serialplot param 表 mvacc 可在线调）

/* ==================== 分段PID参数表（每段一组 kp/ki/kd） ==================== */
typedef struct{
  float kp;  // 比例增益
  float ki;  // 积分增益
  float kd;  // 微分增益（速度环一般置0）
}SpeedSegParam;

/* ==================== 车体结构体 ==================== */
/* 目标速度 = speed_pid[i].target，实测速度 = speed_pid[i].actual，PWM 输出 = speed_pid[i].out */
typedef struct {
    PID_INC  speed_pid[5];       // 4轮速度环（增量式；索引0不用；1~4 对应 TIM1_CH1~4）
    SpeedSegParam speed_seg[SPEED_SEG_NUM];  // 分段PID参数表（控制循环按 target 所在区间取对应段）
    float v_x;                   // 整车x方向速度 cm/s（右移为正）
    float v_y;                   // 整车y方向速度 cm/s（前进为正）
    float w;                     // 整车角速度 rad/s（逆时针为正）
    uint8_t  stop_flag;          // 停车标志：1 时停止输出
    /* 航向环（角度环）：flag.angle=1 时 w 由角度环输出接管（默认开启） */
    PID_POS  yaw_pid;            // 航向环：target=目标角度、actual=实测角度、out=输出w(rad/s)
    float    target_yaw;         // 航向环目标角度（HWT101CT 0~360°，哨兵锁定当前朝向）
    float    yaw;                // 航向环实际角度（镜像 HWT101CT_Data.yaw）
    /* 里程计（④移植自旧代码 RobotCalculate：本车麦轮正解 + 标准旋转矩阵，位置全局坐标） */
    float    pos_x;              // 全局x坐标 cm（右移为正）
    float    pos_y;              // 全局y坐标 cm（前进为正）
    float    now_v_x;            // 当前整车x速度 cm/s
    float    now_v_y;            // 当前整车y速度 cm/s
    float    now_the;            // 当前朝向角 rad（车头相对全局x轴，逆时针为正）
    /* 梯形速度规划（go_to_xy 移植：串口 mx/my 走固定距离自动停，时间开环照搬旧代码） */
    TrapeVelprofile_t tp_x;          // x方向梯形规划曲线（起点静止、末速0、目标 move_speed）
    TrapeVelprofile_t tp_y;          // y方向梯形规划曲线
    uint8_t  x_speed_plan_flag;      // x轴规划进行中（1规划中，0结束）
    uint8_t  y_speed_plan_flag;      // y轴规划进行中
    uint8_t  x_set_speed_flag;       // x轴手动设速标志（串口 vx 置1：中断不清零 v_x）
    uint8_t  y_set_speed_flag;       // y轴手动设速标志（串口 vy 置1）
    float    speed_dir_x;            // x方向符号（+1右移 / -1左移）
    float    speed_dir_y;            // y方向符号（+1前进 / -1后退）
    float    ti;                     // 规划计时 s（10ms 控制周期每周期累加）
    float    move_speed;             // 规划目标速度 cm/s（param 表 mv 可调）
    float    move_acc;               // 规划加减速 cm/s²（param 表 mvacc 可调）
    /* 后续再加：位置闭环x_pid/y_pid等 */
} Chassis;

extern Chassis chassis;

void CHASSIS_Init(void);
void CHASSIS_Control_Loop(void);
void CHASSIS_Mecanum(void);      // 整车速度(v_x,v_y,w) → 4轮目标速度
void CHASSIS_Odom_Calculate(const int16_t pulse[5]);  // 里程计：4轮脉冲 → 车体位移 → 全局坐标积分
void CHASSIS_Start_Move(float x_dist, float y_dist, float x_speed, float y_speed, float x_acc, float y_acc);
  // 梯形规划启动（非阻塞）：x/y方向走固定距离 cm，速度/加减速分别指定；走完自动停

#endif
