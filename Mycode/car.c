#include "car.h"
#include "tb6612.h"
#include "tim.h"      /* htim2~htim5(编码器), htim7(1ms) */
#include <math.h>

/* ========== PID 参数(移植自去年F103, 需按新硬件重新整定!) ========== */
#define speed_p   5.2f
#define speed_i   0.03f
#define speed_d   0.0f

#define yaw_p    -0.02f
#define yaw_i    0.0f
#define yaw_d    -0.02f

/* ========== 全局变量 ========== */
Car my_car;
volatile float CAR_Angle = 0.0f;      /* 陀螺仪航向角(度), 使用时由外部更新 */
volatile float ti = 0.0f;
volatile uint8_t x_set_speed_flag = 0;
volatile uint8_t y_set_speed_flag = 0;
volatile uint8_t w_set_flag = 0;
volatile float speed_dir_x = 0.0f;
volatile float speed_dir_y = 0.0f;
volatile uint8_t x_speed_plan_flag = 0;
volatile uint8_t y_speed_plan_flag = 0;
TrapeVelprofile_t tp_x, tp_y;

/* ========== 工具函数 ========== */
static float abs_f(float num){ return (num >= 0) ? num : -num; }

static float limit(float a, float max)
{
  if(a > max)       return max;
  else if(a < -max) return -max;
  else              return a;
}

/* ===================== PID ===================== */

/* 位置式PID(带积分分离+积分限幅) */
void PositionPID_Calculate(Position_PID *pid, const float Target, const float Measure)
{
  #define Integraldead_zone 0.1f
  const float NoWay = 0.005f;

  pid->Err = Target - Measure;
  pid->Output = pid->Kp*pid->Err + pid->Kd*(pid->Err - pid->Last_Err);
  if(pid->Output > -NoWay && pid->Output < NoWay) pid->Output = 0;

#ifdef HAVE_PID_INTEGRAL
  if(abs_f(pid->Err) < Integraldead_zone) pid->index = 0; else pid->index = 1;
  pid->Integral += pid->Ki * pid->Err * pid->index;
  pid->Integral = limit(pid->Integral, pid->I_outputMax);
  pid->Output += pid->Integral;
#endif
  pid->Output = limit(pid->Output, pid->OutputMax);
  pid->Last_Err = pid->Err;
}

/* 增量式PID(速度环用) */
void IncrementalPID_Calculate(Incremental_PID *pid, const float Target, const float Measure)
{
  pid->Err = Target - Measure;
  pid->p_out = pid->Kp * (pid->Err - pid->Last_Err);
  pid->d_out = pid->Kd * (pid->Err - 2.0f*pid->Last_Err + pid->Previous_Err);
  pid->i_out += pid->Ki * pid->Err;
  if(pid->i_out > 10) pid->i_out = 10;
  if(pid->i_out < -10) pid->i_out = -10;
  if(pid->Ki*pid->Err > -0.1f && pid->Ki*pid->Err < 0.1f){
    if(pid->i_out > 0.2f) pid->i_out = 0.2f;
    else if(pid->i_out < -0.2f) pid->i_out = -0.2f;
  }
  pid->Output += pid->p_out + pid->i_out + pid->d_out;
  pid->Output = limit(pid->Output, pid->OutputMax);
  pid->Previous_Err = pid->Last_Err;
  pid->Last_Err = pid->Err;
}

void PositionPID_clear(Position_PID *pid){ pid->Err = 0; pid->Last_Err = 0; pid->Output = 0; }

/* ===================== 麦轮运动学 ===================== */
#define HALF_LENGTH  (MECANUM_CAR_length/2.0f)   // 轴距一半
#define HALF_WIDTH   (MECANUM_CAR_width/2.0f)    // 轮距一半

/* 逆运动学: 由车体速度求4个轮子目标线速度(cm/s)
 *   v_y 前进+(cm/s)  v_x 横移,右+(cm/s)  w 逆时针+(rad/s)
 *   motor_1~4 = 左前,右前,左后,右后 */
void CAR_Mecanum(double v_y, double v_x, double w)
{
  my_car.motor_1.target_speed = (float)( v_y + v_x - w*(HALF_LENGTH + HALF_WIDTH));
  my_car.motor_2.target_speed = (float)( v_y - v_x + w*(HALF_LENGTH + HALF_WIDTH));
  my_car.motor_3.target_speed = (float)( v_y - v_x - w*(HALF_LENGTH + HALF_WIDTH));
  my_car.motor_4.target_speed = (float)( v_y + v_x + w*(HALF_LENGTH + HALF_WIDTH));
}

/* 极坐标式: v-速度 the-方向角(0前,90左,180后,270右) w-旋转(度/s) */
void CAR_Mecanum_Polar(double v, double the, double w)
{
  double the_mod = fmod(the, 360.0);
  double rad = dev2rad(the_mod);
  double v_x = v * cos(rad);
  double v_y = v * sin(rad);
  CAR_Mecanum(v_y, v_x, dev2rad(w));
}

/* 里程计正解算: 由编码器增量求车体位移/速度 */
void CAR_Robot_Calculate(void)
{
  float deltacounts[4] = {0,0,0,0};
  float delta_x_o, delta_y_o, delta_x, delta_y, everycount;

  my_car.now_the = (360.0f - my_car.yaw) * Pi / 180.0f;
  while(my_car.now_the >= 2.0f*Pi || my_car.now_the < 0){
    if(my_car.now_the >= 2.0f*Pi) my_car.now_the -= 2.0f*Pi;
    if(my_car.now_the < 0)        my_car.now_the += 2.0f*Pi;
  }

  /* 每个脉冲对应的行走距离cm */
  everycount = ((1.0f/ENCODER_ACCURACY)*Tooth_Proportion)*Perimeter;

  deltacounts[0] = my_car.motor_1.encoder_count_f;
  deltacounts[1] = my_car.motor_2.encoder_count_f;
  deltacounts[2] = my_car.motor_4.encoder_count_f;
  deltacounts[3] = my_car.motor_3.encoder_count_f;

  delta_x_o = (-deltacounts[1] + deltacounts[2]) / 2.0f * everycount;
  delta_y_o = (deltacounts[0] + deltacounts[1]) / 2.0f * everycount;

  if(my_car.now_the >= 0 && my_car.now_the < (Pi/2.0f)){
    delta_x = delta_x_o * sin(Pi/2.0f - my_car.now_the) - delta_y_o * sin(my_car.now_the);
    delta_y = delta_x_o * cos(Pi/2.0f - my_car.now_the) + delta_y_o * cos(my_car.now_the);
  }
  if(my_car.now_the >= (Pi/2.0f) && my_car.now_the < Pi){
    delta_x = -(delta_x_o * sin(my_car.now_the - Pi/2.0f) + delta_y_o * cos(my_car.now_the - (Pi/2.0f)));
    delta_y = delta_x_o * cos(my_car.now_the - Pi/2.0f) - delta_y_o * sin(my_car.now_the - (Pi/2.0f));
  }
  if(my_car.now_the >= Pi && my_car.now_the < (3.0f*Pi/2.0f)){
    delta_x = -delta_x_o * cos(my_car.now_the - Pi) + delta_y_o * cos(3.0f*Pi/2.0f - my_car.now_the);
    delta_y = -(delta_x_o * sin(my_car.now_the - Pi) + delta_y_o * sin(3.0f*Pi/2.0f - my_car.now_the));
  }
  if(my_car.now_the >= (3.0f*Pi/2.0f) && my_car.now_the < (2.0f*Pi)){
    delta_x = delta_x_o * sin(my_car.now_the - 3.0f*Pi/2.0f) + delta_y_o * cos(my_car.now_the - 3.0f*Pi/2.0f);
    delta_y = -delta_x_o * cos(my_car.now_the - 3.0f*Pi/2.0f) + delta_y_o * sin(my_car.now_the - 3.0f*Pi/2.0f);
  }

  my_car.now_v_x = delta_x / ENCODER_TIME_S;
  my_car.now_v_y = delta_y / ENCODER_TIME_S;
  my_car.now_x += delta_x;
  my_car.now_y += delta_y;
  my_car.now_point[0] = my_car.now_x;
  my_car.now_point[1] = my_car.now_y;
}

/* ===================== 航向闭环 ===================== */
void CAR_Yaw_PID(void)
{
  float temp_err, temp_yaw;
  my_car.yaw = CAR_Angle;
  temp_err = my_car.target_yaw - my_car.yaw;
  /* 取最短路径 */
  if(temp_err > 180.0f)       temp_err = -360.0f + temp_err;
  else if(temp_err < -180.0f) temp_err = 360.0f + temp_err;
  temp_yaw = temp_err + my_car.yaw;
  PositionPID_Calculate(&my_car.yaw_pid, temp_yaw, my_car.yaw);
  my_car.w = my_car.yaw_pid.Output;
}

/* ===================== 电机 ===================== */
/* 速度环PID */
void CAR_Motor_PID(Motor *motor)
{
  IncrementalPID_Calculate(&motor->s_pid, motor->target_speed, motor->speed);
  motor->PWM = (int32_t)motor->s_pid.Output;
}

/* PWM输出: ±1000 -> ±100% 给 TB6612 */
void CAR_Motor_Control(Motor *motor)
{
  motor->PWM = (int32_t)limit((float)motor->PWM, (float)MY_PWM_MAX);
  int32_t duty = motor->PWM * motor->motor_dir;      /* 方向归一化 */
  if(duty > -motor_dead && duty < motor_dead) duty = 0;   /* 死区 */
  int8_t speed = (int8_t)(duty / (MY_PWM_MAX/100));  /* ±1000 -> ±100 */
  TB6612_Control(motor->motor_id, speed);
}

/* 编码器计数 -> 速度 cm/s */
void CAR_Speed_Translation(Motor *motor)
{
  motor->speed = ((motor->encoder_count_f/ENCODER_ACCURACY)*Tooth_Proportion)*Perimeter/ENCODER_TIME_S;
}

/* 读取4路编码器(10ms周期内计数, 读出后清零) */
void CAR_Encoder_Get(void)
{
  /* 编码器映射: A=TIM2(PA5,PB3) B=TIM3(PA6,PA7) C=TIM4(PD12,PD13) D=TIM5(PA0,PA1) */
  my_car.motor_1.encoder_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  my_car.motor_2.encoder_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  my_car.motor_3.encoder_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
  __HAL_TIM_SET_COUNTER(&htim4, 0);
  my_car.motor_4.encoder_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim5);
  __HAL_TIM_SET_COUNTER(&htim5, 0);

  my_car.motor_1.encoder_count_r = my_car.motor_1.encoder_count * my_car.motor_1.encoder_dir;
  my_car.motor_2.encoder_count_r = my_car.motor_2.encoder_count * my_car.motor_2.encoder_dir;
  my_car.motor_3.encoder_count_r = my_car.motor_3.encoder_count * my_car.motor_3.encoder_dir;
  my_car.motor_4.encoder_count_r = my_car.motor_4.encoder_count * my_car.motor_4.encoder_dir;

  my_car.motor_1.encoder_count_f = my_car.motor_1.encoder_count_r;
  my_car.motor_2.encoder_count_f = my_car.motor_2.encoder_count_r;
  my_car.motor_3.encoder_count_f = my_car.motor_3.encoder_count_r;
  my_car.motor_4.encoder_count_f = my_car.motor_4.encoder_count_r;
}

/* ===================== 初始化 ===================== */
void CAR_Init(void)
{
  uint8_t i;
  Motor *m[4] = {&my_car.motor_1, &my_car.motor_2, &my_car.motor_3, &my_car.motor_4};

  /* 驱动板: STBY=1 + 启动 TIM1 四路PWM */
  TB6612_Init();

  /* 启动4路编码器接口 */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  /* 启动 1ms 定时器中断 TIM7 (中断里每10ms调用 CAR_Control_Loop) */
  HAL_TIM_Base_Start_IT(&htim7);

  /* ========== 电机配置 ==========
   * motor_1~4 = 左前/右前/左后/右后 (按实际装车调整!)
   * motor_id  : 对应 tb6612.h 的 MOTOR_A/B/C/D
   * motor_dir : 正命令对应的实际转向, 若某轮反向则改为 -1
   * encoder_dir: 前进时编码器计数值应>0, 反之改 -1
   */
  my_car.motor_1.motor_id = MOTOR_A;  my_car.motor_1.pwm_ch = 1;
  my_car.motor_1.motor_dir = 1;       my_car.motor_1.encoder_dir = 1;
  my_car.motor_2.motor_id = MOTOR_B;  my_car.motor_2.pwm_ch = 2;
  my_car.motor_2.motor_dir = 1;       my_car.motor_2.encoder_dir = 1;
  my_car.motor_3.motor_id = MOTOR_C;  my_car.motor_3.pwm_ch = 3;
  my_car.motor_3.motor_dir = 1;       my_car.motor_3.encoder_dir = 1;
  my_car.motor_4.motor_id = MOTOR_D;  my_car.motor_4.pwm_ch = 4;
  my_car.motor_4.motor_dir = 1;       my_car.motor_4.encoder_dir = 1;

  for(i = 0; i < 4; i++){
    m[i]->target_speed = 0.0f;
    m[i]->speed = 0.0f;
    m[i]->PWM = 0;
    m[i]->encoder_count_all = 0;
    m[i]->encoder_count_all_last = 0;
    m[i]->s_pid.Kp = speed_p;
    m[i]->s_pid.Ki = speed_i;
    m[i]->s_pid.Kd = speed_d;
    m[i]->s_pid.OutputMax = (float)MY_PWM_MAX;
    m[i]->s_pid.Output = 0.0f;
    m[i]->s_pid.i_out = 0.0f;
    m[i]->s_pid.p_out = 0.0f;
    m[i]->s_pid.d_out = 0.0f;
  }

  /* 航向环PID */
  my_car.yaw_pid.Kp = yaw_p;
  my_car.yaw_pid.Ki = yaw_i;
  my_car.yaw_pid.Kd = yaw_d;
  my_car.yaw_pid.OutputMax = 2.8f;
  my_car.yaw_pid.I_outputMax = 0.5f;
  PositionPID_clear(&my_car.yaw_pid);

  /* 车体状态 */
  my_car.stop_flag = 0;
  my_car.the = 90.0f;
  my_car.speed = 0.0f;
  my_car.w = 0.0f;
  my_car.v_x = 0.0f;
  my_car.v_y = 0.0f;
  my_car.now_v_x = 0.0f;
  my_car.now_v_y = 0.0f;
  my_car.target_yaw = 0.0f;
  my_car.yaw = 0.0f;
  my_car.now_x = 0.0;
  my_car.now_y = 0.0;
  my_car.now_point[0] = 0.0;
  my_car.now_point[1] = 0.0;
  my_car.target_point[0] = 0.0;
  my_car.target_point[1] = 0.0;

  ti = 0.0f;
  x_speed_plan_flag = 0;
  y_speed_plan_flag = 0;
  x_set_speed_flag = 0;
  y_set_speed_flag = 0;
  w_set_flag = 0;
  speed_dir_x = 0.0f;
  speed_dir_y = 0.0f;
}

/* ===================== 10ms 控制周期 ===================== */
void CAR_Control_Loop(void)
{
  CAR_Encoder_Get();

  ti += ENCODER_TIME_S;
  if(ti >= tp_x.t) x_speed_plan_flag = 0;   /* x轴速度规划完成 */
  if(ti >= tp_y.t) y_speed_plan_flag = 0;   /* y轴速度规划完成 */

  if(x_speed_plan_flag) my_car.v_x = speed_dir_x * (float)calcTrapezoidalVel(&tp_x, ti);
  else if(!x_set_speed_flag) my_car.v_x = 0;
  if(y_speed_plan_flag) my_car.v_y = speed_dir_y * (float)calcTrapezoidalVel(&tp_y, ti);
  else if(!y_set_speed_flag) my_car.v_y = 0;

  CAR_Speed_Translation(&my_car.motor_1);
  CAR_Speed_Translation(&my_car.motor_2);
  CAR_Speed_Translation(&my_car.motor_3);
  CAR_Speed_Translation(&my_car.motor_4);

  CAR_Robot_Calculate();
  if(!w_set_flag) CAR_Yaw_PID();            /* 航向闭环, 或用外部直接给w */

  CAR_Mecanum(my_car.v_y, my_car.v_x, my_car.w);   /* 逆运动学 -> 各轮目标速度 */

  if(!my_car.stop_flag){
    CAR_Motor_PID(&my_car.motor_1); CAR_Motor_PID(&my_car.motor_2);
    CAR_Motor_PID(&my_car.motor_3); CAR_Motor_PID(&my_car.motor_4);
    CAR_Motor_Control(&my_car.motor_1); CAR_Motor_Control(&my_car.motor_2);
    CAR_Motor_Control(&my_car.motor_3); CAR_Motor_Control(&my_car.motor_4);
  } else {
    my_car.motor_1.PWM = 0; my_car.motor_2.PWM = 0;
    my_car.motor_3.PWM = 0; my_car.motor_4.PWM = 0;
    CAR_Motor_Control(&my_car.motor_1); CAR_Motor_Control(&my_car.motor_2);
    CAR_Motor_Control(&my_car.motor_3); CAR_Motor_Control(&my_car.motor_4);
  }
}

