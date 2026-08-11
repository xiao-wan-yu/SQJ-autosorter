/**
  ******************************************************************************
  * @file    motor.h
  * @brief   从 F103(10.17晚备战第二场) 移植的迈克纳姆轮四电机驱动
  *          —— 已适配 F407 + HAL 库 + TB6612 双路 H 桥 + TIM1 四路 PWM
  *
  * 硬件映射(相对 F103 的改动):
  *   PWM:   F103 TIM8 CH1~CH4  ->  F407 TIM1 CH1~CH4 (PE9/PE11/PE13/PE14)
  *   DIR:   F103 单 GPIO(PD3~6) ->  F407 TB6612 双输入引脚(IN1/IN2)
  *          电机1: AIN1=PA3  AIN2=PA4
  *          电机2: BIN1=PC4  BIN2=PC5
  *          电机3: CIN1=PB0  CIN2=PB1
  *          电机4: DIN1=PB2  DIN2=PE7
  *          TB6612 STBY=PE8 (置高使能)
  *  编码器: F103 TIM2/3/4/5 -> F407 TIM2(PA5,PB3) TIM3(PA6,PA7)
  *          TIM4(PD12,PD13) TIM5(PA0,PA1)  (CubeMX 已配置为编码器接口)
  *  控制环: F103 TIM6 10ms  ->  F407 复用 TIM7 1ms 中断,每 10 次调用 time_period_fun()
  *
  * 使用说明:
  *   1. main() 初始化后调用 motor_init();
  *   2. 在 HAL_TIM_PeriodElapsedCallback 中 htim==&htim7 时,
  *      每累计 10 次 (10ms) 调用一次 time_period_fun();
  *   3. 遥控/逻辑层直接修改 my_car.v_y(前后 cm/s)、my_car.v_x(左右 cm/s)、
  *      my_car.w(旋转 rad/s, 注意 w_set_flag==0 时由 yaw 环自动给出);
  *   4. 若使用 yaw 闭环,需在每帧把姿态传感器角度(°)写入 my_car.yaw,
  *      target_yaw 为目标角度, w_set_flag=1 可跳过 yaw 环直接手动控 w。
  *   5. ENCODER_ACCURACY / MECANUM_CAR_length / MECANUM_CAR_width 等参数
  *      需按 F407 实际车体和编码器重新标定。
  ******************************************************************************
  */
#ifndef __MOTOR_26_H
#define __MOTOR_26_H

#include "stm32f4xx_hal.h"

/*-------------------------------角度/弧度换算-------------------------------*/
#define rad2dev(rad)  ((rad) * 180.0f / Pi)
#define dev2rad(dev)  ((dev) * Pi / 180.0f)

/*-------------------------------车体参数------------------------------------*/
//车子尺寸需要重新调整，22.5，35.0，13.0已调整
#define MECANUM_CAR_length   22.5f   // 车体长度 (cm) (前后轮轴距)
#define MECANUM_CAR_width    35.0f   // 车体宽度 (cm)
#define WHEEL_DIAMETE        13.0f    // 轮子直径 (cm)
#define Pi                   3.14159265359f   // 圆周率

//编码器电机复用去年的，不需要改动
#define ENCODER_TIME_S       0.010f  // 编码器采样/控制周期 (s)
#define ENCODER_TIME_MS      10.0f   // 编码器采样/控制周期 (ms)
//↑上述两者定时器（TIM7）每隔多长时间（10ms）读取一次编码器并运行一次 PID 控制算法。
#define ENCODER_ACCURACY     896.0f  // 编码器精度(每转脉冲数)
//↑电机转动一圈，编码器输出的总脉冲数（编码器线数 * 减速比 * 4倍频），电机旋转一圈产生的脉冲数保持不变

#define ENCODER_GEAR         1.0f    // 编码器侧齿轮齿数(与轮子齿数比值)
#define WHEEL_GEAR           1.0f    // 轮子齿轮齿数
#define Tooth_Proportion     (ENCODER_GEAR / WHEEL_GEAR)  // 传动比
#define Perimeter            (Pi * WHEEL_DIAMETE)         // 轮子周长 (cm)

#define MY_CURRENT_MAX       (3.0f)  // 电流限幅 (未启用电流环)
#define MY_PWM_MAX           (1000)  // PWM 限幅 (对应 TIM1: PSC=167, ARR=999, duty 0~1000)

#define HAVE_PID_INTEGRAL

/*-------------------------------位置式 PID----------------------------------*/
typedef struct{
  float Kp;
  float Ki;
  float Kd;
#ifdef HAVE_PID_INTEGRAL
  int index;            // 积分分离系数
  float Integral;       // 积分项
  float I_outputMax;    // 积分限幅
#endif
  float Err;
  float Last_Err;       // 上次误差
  float Output;         // PID 输出
  float OutputMax;      // 位置式 PID 输出限幅
}Position_PID;

/*-------------------------------增量式 PID----------------------------------*/
typedef struct{
  float Kp;
  float Ki;
  float Kd;
  float p_out;
  float i_out;
  float d_out;
  float Err;
  float Last_Err;       // 上次误差
  float Previous_Err;   // 上上次误差
  float Output;
  float OutputMax;      // 增量式 PID 输出限幅
}Incremental_PID;

/*-------------------------------单电机结构体---------------------------------*/
typedef struct {
  // PWM/方向初始化参数
  GPIO_TypeDef* dir_GPIOx;   // 方向 IN1 引脚
  uint16_t dir_GPIO_Pin;
  GPIO_TypeDef* dir2_GPIOx;  // 方向 IN2 引脚 (TB6612)
  uint16_t dir2_GPIO_Pin;
  uint8_t pwm_ch;            // PWM 通道 1~4 (TIM1_CH1~CH4)
  uint16_t current_pin;      // 电流 ADC 引脚 (未启用)
  int8_t motor_dir;          // 电机方向矫正

  volatile float target_speed; // 目标速度 (cm/s)
  volatile float speed;        // 实时速度 (cm/s)
  volatile float current;      // 实时电流
  int32_t PWM;                 // PWM 值

  Incremental_PID s_pid;       // 速度环 PID
  Incremental_PID c_pid;       // 电流环 PID (未启用)

  // 编码器参数
  uint16_t A_pin;              // A 相引脚
  uint16_t B_pin;              // B 相引脚
  int8_t encoder_dir;          // 编码器方向矫正

  int32_t encoder_count_all_last; // 编码器上次累计值
  int32_t encoder_count_all;      // 编码器累计值
  int32_t encoder_count;          // 编码器本次增量 (10ms)
  int32_t encoder_count_r;        // 编码器增量(方向矫正后)
  float encoder_count_f;          // 编码器滤波后增量

  float pressure;                 // 轮子承受压力 (kg)
}Motor;

/*-------------------------------小车结构体-----------------------------------*/
typedef struct{
  Motor motor_1,motor_2,motor_3,motor_4; // 左前 右前 左后 右后

  volatile float target_yaw;   // 目标角度 (°)
  volatile float speed;        // 小车目标速度 (cm/s)
  volatile float the;          // 运动方向 (0°为右, 90°为前, 180°为左, 270°为后)
  volatile float w;            // 目标角速度 (rad/s)
  volatile float v_x;          // 左右速度 (右为正) (cm/s)
  volatile float v_y;          // 前后速度 (前为正) (cm/s)
  double target_point[2];      // 目标位置 (x,y)

  volatile float now_v_x;      // 当前左右速度 (cm/s)
  volatile float now_v_y;      // 当前前后速度 (cm/s)
  volatile float yaw;          // 当前角度 (°)——从姿态传感器写入
  volatile double now_point[2];// 当前位置 (x,y)
  volatile double now_x;       // 当前 x 坐标
  volatile double now_y;       // 当前 y 坐标
  volatile float now_the;      // 当前方向角 (rad)

  Position_PID yaw_pid;        // 转向 PID
  Position_PID x_pid,y_pid;    // 位置环 PID

  uint8_t stop_flag;
}Car;

/*-------------------------------外部变量-------------------------------------*/
extern Car my_car;
extern volatile uint8_t w_set_flag; // 1: 跳过 yaw 环,直接使用 my_car.w

/*-------------------------------函数声明-------------------------------------*/
void motor_init(void);                   // 总初始化: TIM1 PWM + TB6612方向引脚 + 编码器启动 + car_init
void car_init(void);                     // 小车结构与 PID 参数初始化
void encoder_count_get(void);            // 读取 4 路编码器增量 (10ms 一次)
void speed_translation(Motor *motor);    // 编码器计数 -> 线速度 (cm/s)
void motor_pid(Motor *motor);            // 速度环增量式 PID
void motor_control(Motor *motor);        // PWM + TB6612 方向输出
void mecanum(double v_y,double v_x,double w);       // 迈克纳姆逆运动学 (xy坐标系)
void mecanum_polar(double v,double the,double w);   // 迈克纳姆逆运动学 (极坐标系)
void car_yaw_pid(void);                  // 角度闭环 PID
void RobotCalculate(void);               // 正运动学解算 (编码器 -> 位置)
void stop_car(void);                     // 停车
void time_period_fun(void);              // 10ms 控制环(由 1ms 中断每10次调用一次)
void PositionPID_Calculate(Position_PID *pid,const float Target,const float Measure);
void IncrementalPID_Calculate(Incremental_PID *pid,const float Target,const float Measure);
void PositionPID_clear(Position_PID *pid);
float abs_f(float num);
float limit(float a,float max);

#endif
