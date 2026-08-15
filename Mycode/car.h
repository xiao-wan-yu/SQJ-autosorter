#ifndef __CAR_H
#define __CAR_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* =====================================================================
 *  麦轮小车: 四轮驱动 + PID闭环 + 麦轮运动学 + 里程计
 *  移植自去年 F103 代码 (待移植/10.17晚备战第二场/User/drivers)
 *  依赖: tb6612.c(四路驱动) tim.c(TIM1 PWM + TIM2/3/4/5编码器 + TIM7 1ms)
 * ===================================================================== */

/* ---------------------- 车体参数(按实际车体修改) ---------------------- */
#define MECANUM_CAR_length   22.5f   // 前后轮轴距 cm
#define MECANUM_CAR_width    35.0f   // 左右轮距 cm
#define WHEEL_DIAMETE        13.0f    // 轮径 cm
#define Pi                   3.14159265359f

/* ---------------------- 编码器/速度换算参数 ---------------------- */
#define ENCODER_TIME_S       0.010f  // 控制周期 s (10ms)
#define ENCODER_TIME_MS      10.0f
#define ENCODER_ACCURACY     896.0f  // 编码器每转一圈脉冲数(线数x倍频)
                                     // !!! 当前编码器定时器是 TI1 模式(2倍频),
                                     //     该值必须按实际电机实测标定 !!!
#define ENCODER_GEAR         1.0f    // 电机齿轮比
#define WHEEL_GEAR           1.0f
#define Tooth_Proportion     (ENCODER_GEAR/WHEEL_GEAR)
#define Perimeter            (Pi*WHEEL_DIAMETE)

/* ---------------------- PWM 输出参数 ---------------------- */
#define MY_PWM_MAX           1000    // 内部速度环输出限幅 ±1000
#define motor_dead           200     // 死区: |duty|<200 不输出 (即<20%不驱动)

/* 位置式PID是否带积分项 */
#define HAVE_PID_INTEGRAL

/* 角度/弧度换算 */
#define rad2dev(rad)  ((rad)*180.0/Pi)
#define dev2rad(dev)  ((dev)*Pi/180.0)

/* ---------------------- 位置式PID ---------------------- */
typedef struct {
  float Kp, Ki, Kd;
#ifdef HAVE_PID_INTEGRAL
  int   index;
  float Integral;
  float I_outputMax;
#endif
  float Err;
  float Last_Err;
  float Output;
  float OutputMax;
} Position_PID;

/* ---------------------- 增量式PID ---------------------- */
typedef struct {
  float Kp, Ki, Kd;
  float p_out, i_out, d_out;
  float Err;
  float Last_Err;
  float Previous_Err;
  float Output;
  float OutputMax;
} Incremental_PID;

/* ---------------------- 电机 ---------------------- */
typedef struct {
  uint8_t motor_id;        // MOTOR_A/B/C/D (对应 tb6612.h)
  int8_t  motor_dir;       // 电机方向归一化 +1/-1 (正命令=前进)
  int8_t  encoder_dir;     // 编码器方向归一化 +1/-1 (前进时计数值为正)
  uint8_t pwm_ch;          // 1~4 (调试用)
  float   target_speed;    // 目标速度 cm/s
  float   speed;           // 实际速度 cm/s
  int32_t PWM;             // 速度环输出 ±1000
  Incremental_PID s_pid;   // 速度环PID
  int32_t encoder_count_all;
  int32_t encoder_count_all_last;
  int32_t encoder_count;
  int32_t encoder_count_r;
  float   encoder_count_f;
} Motor;

/* ---------------------- 车体 ---------------------- */
typedef struct {
  Motor motor_1, motor_2, motor_3, motor_4;   // 左前,右前,左后,右后
  volatile float target_yaw;   // 目标航向角 (度)
  volatile float speed;        // 车体速度
  volatile float the;          // 车体朝向 (度)
  volatile float w;            // 旋转角速度 (rad/s, 逆时针+)
  volatile float v_x;          // 横向速度 (cm/s, 右+)
  volatile float v_y;          // 纵向速度 (cm/s, 前+)
  volatile float now_v_x, now_v_y;   // 里程计当前速度
  volatile float yaw;          // 当前航向角 (度, 由陀螺仪更新)
  volatile double now_point[2];
  volatile double now_x, now_y;
  volatile float now_the;
  volatile double target_point[2];
  Position_PID yaw_pid;        // 航向闭环PID
  uint8_t stop_flag;           // 1=停机
} Car;

/* ---------------------- 梯形速度规划结构体 ---------------------- */
typedef struct {
  double L, t1, t2, t3, t;
  double L1, L2, L3;
  double vs, vc, ve;
  double acc, dec;
} TrapeVelprofile_t;

extern Car my_car;
extern volatile float CAR_Angle;   // 车体当前航向(度), 由陀螺仪更新
extern volatile float ti;          // 速度规划计时
extern volatile uint8_t x_set_speed_flag, y_set_speed_flag, w_set_flag;
extern volatile float speed_dir_x, speed_dir_y;

/* ---------------------- 函数声明 ---------------------- */
void CAR_Init(void);                    // 初始化:TB6612+编码器+TIM7+PID参数
void CAR_Control_Loop(void);            // 10ms控制周期入口(TIM7中断中调用)
void CAR_Mecanum(double v_y, double v_x, double w);        // 麦轮逆运动学
void CAR_Mecanum_Polar(double v, double the, double w);    // 极坐标式
void CAR_Robot_Calculate(void);         // 里程计正解算
void CAR_Yaw_PID(void);                 // 航向闭环PID
void CAR_Motor_PID(Motor *motor);       // 单电机速度环PID
void CAR_Motor_Control(Motor *motor);   // 单电机PWM输出
void CAR_Speed_Translation(Motor *motor);  // 编码器计数->速度cm/s
void CAR_Encoder_Get(void);             // 读取4路编码器

void PositionPID_Calculate(Position_PID *pid, const float Target, const float Measure);
void IncrementalPID_Calculate(Incremental_PID *pid, const float Target, const float Measure);
void PositionPID_clear(Position_PID *pid);

/* 梯形速度规划 */
int    calcTrapezoidalProfile(double L, double vs, double vmax, double ve, double amax, double dmax, TrapeVelprofile_t* tp);
double calcTrapezoidalAcc(TrapeVelprofile_t* tp, double t);
double calcTrapezoidalVel(TrapeVelprofile_t* tp, double t);
double calcTrapezoidalDist(TrapeVelprofile_t* tp, double t);

#endif /* __CAR_H */
