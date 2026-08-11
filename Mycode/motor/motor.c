/**
  ******************************************************************************
  * @file    motor.c
  * @brief   从 F103(10.17晚备战第二场) 移植的迈克纳姆轮四电机驱动
  *          —— 已适配 F407 + HAL 库 + TB6612 + TIM1
  * 详细说明见 motor.h
  ******************************************************************************
  */
#include "motor.h"
#include "main.h"
#include <math.h>

/*-------------------------------PID 参数------------------------------------*/
#define speed_p 0.00f   // 速度环 Kp（原5.20）
#define speed_i 0.00f  // 速度环 Ki（原0.03）
#define speed_d 0.00f   // 速度环 Kd（原0.00）

#define yaw_p -0.02f   // 角度环 Kp（原）
#define yaw_i -0.0f    // 角度环 Ki（原）
#define yaw_d -0.02f   // 角度环 Kd（原）

#define yaw_p_k 0.008f
#define yaw_d_k 0.008f

#define x_p 0
#define x_i 0
#define x_d 0

#define y_p 0
#define y_i 0
#define y_d 0

// 角度环 PID 死区/积分参数 (F103 原值)
#define Integraldead_zone 0.1
float NoWay = 0.005;   // 角度环死区 (每次转角度前要关闭再打开,否则会卡住角度)

/*-------------------------------外部句柄------------------------------------*/
extern TIM_HandleTypeDef htim1;   // 电机 PWM (TIM1_CH1~CH4)
extern TIM_HandleTypeDef htim2;   // 编码器 1 (电机1)
extern TIM_HandleTypeDef htim3;   // 编码器 2 (电机2)
extern TIM_HandleTypeDef htim4;   // 编码器 3 (电机3)
extern TIM_HandleTypeDef htim5;   // 编码器 4 (电机4)

/*-------------------------------全局变量------------------------------------*/
Car my_car;
volatile uint8_t w_set_flag = 0;  // 1: 跳过 yaw 环,直接使用 my_car.w
float temp_err = 0.0f;            // yaw 环中间变量 (F103 定义在 bsp_pid.c)
float temp_yaw = 0.0f;

/*-------------------------------工具函数------------------------------------*/
float abs_f(float num)
{
    return (num >= 0) ? num : -num;
}

float limit(float a, float max)
{
    if(a > max)  return  max;
    else if(a < -max) return -max;
    else return a;
}

/**
  * @brief 位置式 PID
  */
void PositionPID_Calculate(Position_PID *pid, const float Target, const float Measure)
{
    pid->Err = Target - Measure;

    pid->Output = pid->Kp * pid->Err + pid->Kd * (pid->Err - pid->Last_Err);
    if(pid->Output > -NoWay && pid->Output < NoWay) pid->Output = 0; // 死区,注意负值

#ifdef HAVE_PID_INTEGRAL
    if(abs_f(pid->Err) < Integraldead_zone)
        pid->index = 0;
    else
        pid->index = 1;

    pid->Integral += pid->Ki * pid->Err * pid->index;
    pid->Integral  = limit(pid->Integral, pid->I_outputMax);
    pid->Output   += pid->Integral;
#endif
    pid->Output = limit(pid->Output, pid->OutputMax);
    pid->Last_Err = pid->Err;
}

/**
  * @brief 增量式 PID
  */
void IncrementalPID_Calculate(Incremental_PID *pid, const float Target, const float Measure)
{
    pid->Err = Target - Measure;

    pid->p_out = pid->Kp * (pid->Err - pid->Last_Err);
    pid->d_out = pid->Kd * (pid->Err - 2.0f*pid->Last_Err + pid->Previous_Err);
    pid->i_out += pid->Ki * pid->Err;
    if(pid->i_out > 10)  pid->i_out = 10;
    if(pid->i_out < -10) pid->i_out = -10;
    if(pid->Ki * pid->Err > -0.1 && pid->Ki * pid->Err < 0.1)
    {
        if(pid->i_out > 0.2)      pid->i_out = 0.2;
        else if(pid->i_out < -0.2) pid->i_out = -0.2;
    }

    pid->Output += pid->p_out + pid->i_out + pid->d_out;
    pid->Output  = limit(pid->Output, pid->OutputMax); // 限幅

    pid->Previous_Err = pid->Last_Err;
    pid->Last_Err     = pid->Err;
}

void PositionPID_clear(Position_PID *pid)
{
    pid->Err = 0;
    pid->Last_Err = 0;
    pid->Output = 0;
}

/*=====================================================================
 * 硬件初始化
 *====================================================================*/
/**
  * @brief TIM1 四路 PWM 初始化 (F103 TIM8 -> F407 TIM1)
  * CubeMX 生成的是 OC 输出模式(TIM_OCMODE_TIMING),这里重配为 PWM1 模式。
  * 同时把 ARR 改为 999 (PSC=167 -> 定时器 1MHz -> 1kHz PWM, duty 0~1000),
  * 与 F103 (Period 999 / MY_PWM_MAX 1000) 的占空比量程保持一致。
  */
static void motor_pwm_init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* 改时基: 168MHz / (167+1) = 1MHz, ARR=999 -> 1kHz, 占空比 0~1000 */
    __HAL_TIM_SET_PRESCALER(&htim1, 167);
    __HAL_TIM_SET_AUTORELOAD(&htim1, 1000 - 1);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    htim1.Instance->EGR = TIM_EGR_UG;   // 立即装载 ARR/PSC

    /* 四路通道重配为 PWM1 */
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4);

    /* 启动 PWM 输出 (高级定时器需启动主输出 MOE) */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

/**
  * @brief TB6612 方向引脚初始化 (CubeMX 已在 MX_GPIO_Init 配置为推挽输出,
  *        这里做兜底初始化并把 STBY 置高使能、IN 全部拉低)
  */
static void motor_dir_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitStruct.Pin = AIN1_Pin | AIN2_Pin;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BIN1_Pin | BIN2_Pin;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = CIN1_Pin | CIN2_Pin | DIN1_Pin;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DIN2_Pin | TB6612_STBY_Pin;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* 使能 TB6612, 全部方向输入拉低 */
    HAL_GPIO_WritePin(TB6612_STBY_GPIO_Port, TB6612_STBY_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, AIN1_Pin | AIN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, BIN1_Pin | BIN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, CIN1_Pin | CIN2_Pin | DIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, DIN2_Pin, GPIO_PIN_RESET);
}

/**
  * @brief 启动 4 路编码器接口 (CubeMX 已把 TIM2/3/4/5 配成编码器模式)
  * 注意: 每 10ms 控制环读取后清零计数器, 因此不需要溢出中断。
  */
static void motor_encoder_start(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    __HAL_TIM_SET_COUNTER(&htim5, 0);
}

/*=====================================================================
 * 小车初始化
 *====================================================================*/
/**
  * @brief 小车结构与 PID 参数初始化
  * 电机编号: 1=左前 2=右前 3=左后 4=右后
  * PWM: TIM1_CH1~CH4 (PE9/PE11/PE13/PE14)
  * 方向: TB6612 IN1/IN2, 正转约定 IN1=0 IN2=1
  * 注意: motor_dir / encoder_dir 需按实际接线和转向重新标定
  */
void car_init(void)
{
    my_car.stop_flag = 0;

    my_car.motor_1.pressure = 0.1f;   // 轮子承受压力 (未启用压力控制)
    my_car.motor_2.pressure = 0.1f;
    my_car.motor_3.pressure = 0.1f;
    my_car.motor_4.pressure = 0.1f;

    /* 电机1 (左前): TIM1_CH1 + AIN1/AIN2 */
    my_car.motor_1.dir_GPIOx  = GPIOA;
    my_car.motor_1.dir_GPIO_Pin  = AIN1_Pin;
    my_car.motor_1.dir2_GPIOx = GPIOA;
    my_car.motor_1.dir2_GPIO_Pin = AIN2_Pin;
    my_car.motor_1.pwm_ch  = 1;
    my_car.motor_1.motor_dir   = 1;
    my_car.motor_1.encoder_dir = -1;
    my_car.motor_1.PWM = 0;
    my_car.motor_1.target_speed = 0.0f;

    /* 电机2 (右前): TIM1_CH2 + BIN1/BIN2 */
    my_car.motor_2.dir_GPIOx  = GPIOC;
    my_car.motor_2.dir_GPIO_Pin  = BIN1_Pin;
    my_car.motor_2.dir2_GPIOx = GPIOC;
    my_car.motor_2.dir2_GPIO_Pin = BIN2_Pin;
    my_car.motor_2.pwm_ch  = 2;
    my_car.motor_2.motor_dir   = -1;
    my_car.motor_2.encoder_dir = 1;
    my_car.motor_2.PWM = 0;
    my_car.motor_2.target_speed = 0.0f;

    /* 电机3 (左后): TIM1_CH3 + CIN1/CIN2 */
    my_car.motor_3.dir_GPIOx  = GPIOB;
    my_car.motor_3.dir_GPIO_Pin  = CIN1_Pin;
    my_car.motor_3.dir2_GPIOx = GPIOB;
    my_car.motor_3.dir2_GPIO_Pin = CIN2_Pin;
    my_car.motor_3.pwm_ch  = 3;
    my_car.motor_3.motor_dir   = 1;
    my_car.motor_3.encoder_dir = -1;
    my_car.motor_3.PWM = 0;
    my_car.motor_3.target_speed = 0.0f;

    /* 电机4 (右后): TIM1_CH4 + DIN1/DIN2 */
    my_car.motor_4.dir_GPIOx  = GPIOB;
    my_car.motor_4.dir_GPIO_Pin  = DIN1_Pin;
    my_car.motor_4.dir2_GPIOx = GPIOE;
    my_car.motor_4.dir2_GPIO_Pin = DIN2_Pin;
    my_car.motor_4.pwm_ch  = 4;
    my_car.motor_4.motor_dir   = -1;
    my_car.motor_4.encoder_dir = 1;
    my_car.motor_4.PWM = 0;
    my_car.motor_4.target_speed = 0.0f;

    /* 速度环 PID 参数 (增量式 PI) */
    my_car.motor_1.s_pid.Kp = speed_p;
    my_car.motor_1.s_pid.Ki = speed_i;
    my_car.motor_1.s_pid.Kd = speed_d;
    my_car.motor_1.s_pid.OutputMax = MY_PWM_MAX;

    my_car.motor_2.s_pid.Kp = speed_p;
    my_car.motor_2.s_pid.Ki = speed_i;
    my_car.motor_2.s_pid.Kd = speed_d;
    my_car.motor_2.s_pid.OutputMax = MY_PWM_MAX;

    my_car.motor_3.s_pid.Kp = speed_p;
    my_car.motor_3.s_pid.Ki = speed_i;
    my_car.motor_3.s_pid.Kd = speed_d;
    my_car.motor_3.s_pid.OutputMax = MY_PWM_MAX;

    my_car.motor_4.s_pid.Kp = speed_p;
    my_car.motor_4.s_pid.Ki = speed_i;
    my_car.motor_4.s_pid.Kd = speed_d;
    my_car.motor_4.s_pid.OutputMax = MY_PWM_MAX;

    /* 角度环 PID 参数 (位置式 PD) */
    my_car.yaw_pid.Kp = yaw_p;
    my_car.yaw_pid.Ki = yaw_i;
    my_car.yaw_pid.Kd = yaw_d;
    my_car.yaw_pid.OutputMax = 2.8f;
    my_car.yaw_pid.I_outputMax = 0.0f;

    /* 位置环 PID (未启用) */
    my_car.x_pid.Kp = x_p; my_car.x_pid.Ki = x_i; my_car.x_pid.Kd = x_d; my_car.x_pid.OutputMax = 0.0f;
    my_car.y_pid.Kp = y_p; my_car.y_pid.Ki = y_i; my_car.y_pid.Kd = y_d; my_car.y_pid.OutputMax = 0.0f;

    /* 运动状态初值 */
    my_car.the = 90.0f;
    my_car.speed = 0.0f;
    my_car.w = 0.0f;
    my_car.v_x = 0.0f;
    my_car.v_y = 0.0f;
    my_car.yaw = 0.0f;
    my_car.target_yaw = 0.0f;

    /* 位置初值 */
    my_car.now_x = 0.0f;
    my_car.now_y = 0.0f;
    my_car.now_point[0] = my_car.now_x;
    my_car.now_point[1] = my_car.now_y;
    my_car.target_point[0] = my_car.now_point[0];
    my_car.target_point[1] = my_car.now_point[1];
}

/**
  * @brief 电机子系统总初始化 (main() 初始化后调用一次)
  */
void motor_init(void)
{
    motor_pwm_init();
    motor_dir_gpio_init();
    motor_encoder_start();
    car_init();
}

/*=====================================================================
 * 编码器与速度
 *====================================================================*/
/**
  * @brief 读取 4 路编码器增量 (每 10ms 调用一次)
  * F407 方案: 直接读取计数器后清零, 得到 10ms 增量, 无需溢出中断。
  * (F103 用 累计值*5120+计数器 方式, 本板 CubeMX 编码器 ARR 不同, 故简化)
  */
void encoder_count_get(void)
{
    int32_t cnt;

    /* 电机1 -> TIM2 */
    cnt = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    my_car.motor_1.encoder_count = cnt;

    /* 电机2 -> TIM3 */
    cnt = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    my_car.motor_2.encoder_count = cnt;

    /* 电机3 -> TIM4 */
    cnt = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    my_car.motor_3.encoder_count = cnt;

    /* 电机4 -> TIM5 */
    cnt = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
    __HAL_TIM_SET_COUNTER(&htim5, 0);
    my_car.motor_4.encoder_count = cnt;

    /* 累计值 + 方向矫正 */
    my_car.motor_1.encoder_count_all   += my_car.motor_1.encoder_count;
    my_car.motor_1.encoder_count_r      = my_car.motor_1.encoder_count * my_car.motor_1.encoder_dir;
    my_car.motor_1.encoder_count_f      = my_car.motor_1.encoder_count_r;   // 滤波值(暂未滤波)

    my_car.motor_2.encoder_count_all   += my_car.motor_2.encoder_count;
    my_car.motor_2.encoder_count_r      = my_car.motor_2.encoder_count * my_car.motor_2.encoder_dir;
    my_car.motor_2.encoder_count_f      = my_car.motor_2.encoder_count_r;

    my_car.motor_3.encoder_count_all   += my_car.motor_3.encoder_count;
    my_car.motor_3.encoder_count_r      = my_car.motor_3.encoder_count * my_car.motor_3.encoder_dir;
    my_car.motor_3.encoder_count_f      = my_car.motor_3.encoder_count_r;

    my_car.motor_4.encoder_count_all   += my_car.motor_4.encoder_count;
    my_car.motor_4.encoder_count_r      = my_car.motor_4.encoder_count * my_car.motor_4.encoder_dir;
    my_car.motor_4.encoder_count_f      = my_car.motor_4.encoder_count_r;

    my_car.motor_1.encoder_count_all_last = my_car.motor_1.encoder_count_all;
    my_car.motor_2.encoder_count_all_last = my_car.motor_2.encoder_count_all;
    my_car.motor_3.encoder_count_all_last = my_car.motor_3.encoder_count_all;
    my_car.motor_4.encoder_count_all_last = my_car.motor_4.encoder_count_all;
}

/**
  * @brief 编码器计数 -> 轮子线速度 (cm/s)
  * speed = 脉冲数/每转脉冲数 * 传动比 * 轮周长 / 采样时间
  */
void speed_translation(Motor *motor)
{
    motor->speed = ((motor->encoder_count_f / ENCODER_ACCURACY) * Tooth_Proportion) * Perimeter / ENCODER_TIME_S;
}

/*=====================================================================
 * 电机闭环控制
 *====================================================================*/
/**
  * @brief 速度环增量式 PID
  */
void motor_pid(Motor *motor)
{
    IncrementalPID_Calculate(&motor->s_pid, motor->target_speed, motor->speed);
    motor->PWM = motor->s_pid.Output;
}

/**
  * @brief PWM + TB6612 方向输出 (F103 单 DIR 引脚 -> F407 双 IN 引脚)
  * 正转约定: IN1=0, IN2=1; 反转: IN1=1, IN2=0 (与现有 tb6612.c 一致)
  * PWM: TIM1_CH1~CH4
  */
void motor_control(Motor *motor)
{
    uint32_t channel;
    int32_t duty;

    /* PWM 限幅 */
    motor->PWM = limit(motor->PWM, MY_PWM_MAX);
    duty = motor->PWM * motor->motor_dir;   // 电机方向矫正

    switch(motor->pwm_ch)
    {
        case 1:  channel = TIM_CHANNEL_1; break;
        case 2:  channel = TIM_CHANNEL_2; break;
        case 3:  channel = TIM_CHANNEL_3; break;
        default: channel = TIM_CHANNEL_4; break;
    }

    if(duty >= 0)   // 正转
    {
        HAL_GPIO_WritePin(motor->dir_GPIOx,  motor->dir_GPIO_Pin,  GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->dir2_GPIOx, motor->dir2_GPIO_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, channel, (uint32_t)duty);
    }
    else            // 反转
    {
        HAL_GPIO_WritePin(motor->dir_GPIOx,  motor->dir_GPIO_Pin,  GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->dir2_GPIOx, motor->dir2_GPIO_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, channel, (uint32_t)(-duty));
    }
}

/*=====================================================================
 * 迈克纳姆轮运动学
 *====================================================================*/
#define HALF_LENGTH  (MECANUM_CAR_length / 2.0f)   // 半轴距
#define HALF_WIDTH   (MECANUM_CAR_width  / 2.0f)   // 半轮距

/**
  * @brief 迈克纳姆逆运动学 (xy 直角坐标系)
  * @param v_y  前后速度 (前为正) (cm/s)
  * @param v_x  左右速度 (右为正) (cm/s)
  * @param w    旋转角速度 (逆时针为正) (rad/s)
  * 电机 1/2/3/4 = 左前/右前/左后/右后
  */
void mecanum(double v_y, double v_x, double w)
{
    my_car.motor_1.target_speed = ( v_y + v_x - w * (HALF_LENGTH + HALF_WIDTH));
    my_car.motor_2.target_speed = ( v_y - v_x + w * (HALF_LENGTH + HALF_WIDTH));
    my_car.motor_3.target_speed = ( v_y - v_x - w * (HALF_LENGTH + HALF_WIDTH));
    my_car.motor_4.target_speed = ( v_y + v_x + w * (HALF_LENGTH + HALF_WIDTH));
}

/**
  * @brief 迈克纳姆逆运动学 (极坐标系)
  * @param v    合速度 (cm/s)
  * @param the  运动方向 (0°为右, 90°为前, 180°为左, 270°为后)
  * @param w    旋转角速度 (°/s)
  */
void mecanum_polar(double v, double the, double w)
{
    double v_y, v_x, rad, the_mod, w_rad;
    the_mod = fmod(the, 360.0);          // 角度取模
    rad     = dev2rad(the_mod);          // 角度 -> 弧度
    v_x     = v * cos(rad);
    v_y     = v * sin(rad);
    w_rad   = dev2rad(w);                // 角度 -> 弧度
    mecanum(v_y, v_x, w_rad);
}

/**
  * @brief 角度闭环 PID (F103 中由 HWT101 提供角度, 本板请把姿态角度写入 my_car.yaw)
  */
void car_yaw_pid(void)
{
    temp_err = my_car.target_yaw - my_car.yaw;

    /* 取最短路径角度差 */
    if(temp_err > 180.0f)          temp_err = -360.0f + temp_err;
    else if(temp_err < -180.0f)    temp_err =  360.0f + temp_err;

    temp_yaw = temp_err + my_car.yaw;

    PositionPID_Calculate(&my_car.yaw_pid, temp_yaw, my_car.yaw);
    my_car.w = my_car.yaw_pid.Output;
}

/**
  * @brief 正运动学解算: 编码器增量 -> 小车位置/速度 (用于定位)
  */
void RobotCalculate(void)
{
    float deltacounts[4] = {0,0,0,0};
    float delta_x_o, delta_y_o;
    float delta_x, delta_y;
    float everycount;

    my_car.now_the = (360 - my_car.yaw) * Pi / 180.0f;

    while((my_car.now_the >= 2.0f * Pi) || (my_car.now_the < 0))
    {
        if(my_car.now_the >= 2.0f * Pi) my_car.now_the -= 2.0f * Pi;
        if(my_car.now_the < 0)          my_car.now_the += 2.0f * Pi;
    }

    everycount = ((1.0f / ENCODER_ACCURACY) * Tooth_Proportion) * Perimeter;  // 每脉冲行走距离 (cm)

    deltacounts[0] = my_car.motor_1.encoder_count_f;
    deltacounts[1] = my_car.motor_2.encoder_count_f;
    deltacounts[2] = my_car.motor_4.encoder_count_f;
    deltacounts[3] = my_car.motor_3.encoder_count_f;

    delta_x_o = (-deltacounts[1] + deltacounts[2]) / 2.0f * everycount;
    delta_y_o = ( deltacounts[0] + deltacounts[1]) / 2.0f * everycount;

    if(my_car.now_the >= 0 && my_car.now_the < (Pi/2.0f))
    {
        delta_x =  delta_x_o * sin(Pi/2.0f - my_car.now_the) - delta_y_o * sin(my_car.now_the);
        delta_y =  delta_x_o * cos(Pi/2.0f - my_car.now_the) + delta_y_o * cos(my_car.now_the);
    }
    else if(my_car.now_the >= (Pi/2.0f) && my_car.now_the < Pi)
    {
        delta_x = -(delta_x_o * sin(my_car.now_the - Pi/2.0f) + delta_y_o * cos(my_car.now_the - Pi/2.0f));
        delta_y =  delta_x_o * cos(my_car.now_the - Pi/2.0f) - delta_y_o * sin(my_car.now_the - Pi/2.0f);
    }
    else if(my_car.now_the >= Pi && my_car.now_the < (3.0f*Pi/2.0f))
    {
        delta_x = -delta_x_o * cos(my_car.now_the - Pi) + delta_y_o * cos(3.0f*Pi/2.0f - my_car.now_the);
        delta_y = -(delta_x_o * sin(my_car.now_the - Pi) + delta_y_o * sin(3.0f*Pi/2.0f - my_car.now_the));
    }
    else
    {
        delta_x =  delta_x_o * sin(my_car.now_the - 3.0f*Pi/2.0f) + delta_y_o * cos(my_car.now_the - 3.0f*Pi/2.0f);
        delta_y = -delta_x_o * cos(my_car.now_the - 3.0f*Pi/2.0f) + delta_y_o * sin(my_car.now_the - 3.0f*Pi/2.0f);
    }

    my_car.now_v_x = delta_x / 0.005f;
    my_car.now_v_y = delta_y / 0.005f;
    my_car.now_x += delta_x;
    my_car.now_y += delta_y;
    my_car.now_point[0] = my_car.now_x;
    my_car.now_point[1] = my_car.now_y;
}

/*=====================================================================
 * 控制环
 *====================================================================*/
/**
  * @brief 停车
  */
void stop_car(void)
{
    my_car.v_y = 0.0f;
    my_car.v_x = 0.0f;
    my_car.w = 0.0f;
}

/**
  * @brief 10ms 控制环 —— 在 1ms 中断(HAL_TIM_PeriodElapsedCallback htim7)中每 10 次调用一次
  * 流程: 读编码器 -> 算速度 -> 正运动学 -> yaw环 -> 逆运动学 -> 速度环 -> 输出PWM
  */
void time_period_fun(void)
{
    encoder_count_get();

    speed_translation(&my_car.motor_1);
    speed_translation(&my_car.motor_2);
    speed_translation(&my_car.motor_3);
    speed_translation(&my_car.motor_4);

    RobotCalculate();   // 正运动学 (编码器 -> 位置)

    if(!w_set_flag) car_yaw_pid();  // 角度闭环 (w_set_flag=1 时手动控 w)

    mecanum(my_car.v_y, my_car.v_x, my_car.w);  // 逆运动学 -> 目标轮速

    if(!my_car.stop_flag)
    {
        motor_pid(&my_car.motor_1);
        motor_pid(&my_car.motor_2);
        motor_pid(&my_car.motor_3);
        motor_pid(&my_car.motor_4);

        motor_control(&my_car.motor_1);
        motor_control(&my_car.motor_2);
        motor_control(&my_car.motor_3);
        motor_control(&my_car.motor_4);
    }
    else
    {
        my_car.motor_1.PWM = 0; my_car.motor_2.PWM = 0;
        my_car.motor_3.PWM = 0; my_car.motor_4.PWM = 0;

        motor_control(&my_car.motor_1);
        motor_control(&my_car.motor_2);
        motor_control(&my_car.motor_3);
        motor_control(&my_car.motor_4);
    }
}

