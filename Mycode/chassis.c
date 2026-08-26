#include <stdint.h>
#include <stm32f4xx_hal.h>
#include "chassis.h"
#include "pid.h"
#include "encoder.h"
#include "tb6612.h"
#include "hwt101ct.h"
#include <stdlib.h>
#include <math.h>

Chassis chassis;

/**
  * @brief 底盘初始化：配置4轮速度环PID参数
  * @attention PWM/编码器/STBY 的启动由 main.c 外设启动区负责
  */
void CHASSIS_Init(void){
  for(uint8_t i = CHASSIS_MOTOR_LF; i <= CHASSIS_MOTOR_RF; i++){
    PID_INC *pid = &chassis.speed_pid[i];
    pid->target   = 0.0f;
    pid->actual   = 0.0f;
    pid->out      = 0.0f;
    pid->kp       = SPEED_PID_KP;
    pid->ki       = SPEED_PID_KI;
    pid->kd       = SPEED_PID_KD;
    pid->p_out    = 0.0f;
    pid->i_out    = 0.0f;
    pid->d_out    = 0.0f;
    pid->err      = 0.0f;
    pid->last_err = 0.0f;
    pid->prev_err = 0.0f;
    pid->out_max  = SPEED_PID_OUT_MAX;
  }
  chassis.stop_flag = 0;
  chassis.v_x = 0.0f;
  chassis.v_y = 0.0f;
  chassis.w   = 0.0f;

  /* 航向环（角度环）：P(PD) 控制器，参数见 chassis.h；target_yaw 哨兵 → 首次控制循环锁定当前朝向 */
  chassis.yaw_pid.target      = 0.0f;
  chassis.yaw_pid.actual      = 0.0f;
  chassis.yaw_pid.out         = 0.0f;
  chassis.yaw_pid.kp          = YAW_PID_KP;
  chassis.yaw_pid.ki          = YAW_PID_KI;
  chassis.yaw_pid.kd          = YAW_PID_KD;
  chassis.yaw_pid.error0      = 0.0f;
  chassis.yaw_pid.error1      = 0.0f;
  chassis.yaw_pid.errorint    = 0.0f;
  chassis.yaw_pid.integral_max = 0.0f;   // 角度环 ki=0，无需积分限幅
  chassis.yaw_pid.out_max     = YAW_PID_OUT_MAX;
  chassis.yaw_pid.out_min     = YAW_PID_OUT_MIN;
  chassis.target_yaw          = YAW_TARGET_NONE;  // 哨兵：未锁定/未遥控时保持当前朝向
  chassis.yaw                 = 0.0f;

  /* 里程计：初始位置/速度为0 */
  chassis.pos_x    = 0.0f;
  chassis.pos_y    = 0.0f;
  chassis.now_v_x  = 0.0f;
  chassis.now_v_y  = 0.0f;
  chassis.now_the  = 0.0f;

  /* 梯形速度规划（go_to_xy 移植）：初始无规划、无手动设速；mv/mvacc 默认值 */
  chassis.ti                  = 0.0f;
  chassis.speed_dir_x         = 1.0f;
  chassis.speed_dir_y         = 1.0f;
  chassis.x_speed_plan_flag   = 0;
  chassis.y_speed_plan_flag   = 0;
  chassis.x_set_speed_flag    = 0;
  chassis.y_set_speed_flag    = 0;
  chassis.move_speed          = MOVE_SPEED_DEFAULT;
  chassis.move_acc            = MOVE_ACC_DEFAULT;

  /* 分段PID参数表（用户实测标定：不同目标速度区间用不同参数，控制循环按 target 选段应用） */
  chassis.speed_seg[0].kp = 0.8f;   chassis.speed_seg[0].ki = 0.025f; chassis.speed_seg[0].kd = 0.0f; // 0-40
  chassis.speed_seg[1].kp = 2.5f;   chassis.speed_seg[1].ki = 0.02f;  chassis.speed_seg[1].kd = 0.0f; // 40-80
  chassis.speed_seg[2].kp = 3.0f;   chassis.speed_seg[2].ki = 0.05f;  chassis.speed_seg[2].kd = 0.0f; // 80-120
  chassis.speed_seg[3].kp = 3.5f;   chassis.speed_seg[3].ki = 0.02f;  chassis.speed_seg[3].kd = 0.0f; // 120-160
}

/**
  * @brief 目标速度对称限幅
  */
static float clamp_speed(float v, float max){
  if(v >  max) return  max;
  if(v < -max) return -max;
  return v;
}

/**
  * @brief 麦轮逆解算：整车速度(v_x,v_y,w) → 4轮目标速度（覆盖 speed_pid[i].target）
  * @note  轮子顺序 1左前 2左后 3右后 4右前（与 TB6612/编码器/TIM1 通道一致）
  *        v_x 右移为正、v_y 前进为正、w 逆时针为正(rad/s)
  *        已按本车辊子方向实测重排：
  *        v_x：对角轮同向（LF/RB 同向、RF/LB 反向）→ 平移
  *        w ：同侧轮同向（LF/LB 同向、RF/RB 反向）→ 原地旋转
  *        L/W 用实测值(22.8/33.5cm)；对4轮目标做 ±SPEED_TARGET_MAX 限幅防超限
  */
void CHASSIS_Mecanum(void){
  float half_sum = (MECANUM_LENGTH + MECANUM_WIDTH) / 2.0f;
  chassis.speed_pid[CHASSIS_MOTOR_LF].target = clamp_speed( chassis.v_y + chassis.v_x - chassis.w * half_sum, SPEED_TARGET_MAX);
  chassis.speed_pid[CHASSIS_MOTOR_LB].target = clamp_speed( chassis.v_y - chassis.v_x - chassis.w * half_sum, SPEED_TARGET_MAX);
  chassis.speed_pid[CHASSIS_MOTOR_RB].target = clamp_speed( chassis.v_y + chassis.v_x + chassis.w * half_sum, SPEED_TARGET_MAX);
  chassis.speed_pid[CHASSIS_MOTOR_RF].target = clamp_speed( chassis.v_y - chassis.v_x + chassis.w * half_sum, SPEED_TARGET_MAX);
}

/**
  * @brief 里程计：4轮本周期编码器脉冲 → 车体位移(本车麦轮正解) → 旋转到全局坐标 → 速度/位置积分
  * @param pulse 4轮本周期编码器脉冲数（清零法，前进为正；数组索引1~4=LF/LB/RB/RF）
  * @note  移植自旧代码 RobotCalculate，数学重构：
  *        1) 车体位移用本车麦轮正解（与 CHASSIS_Mecanum 逆解严格互逆，见下），而非旧代码的部分轮子组合
  *           dvy=(dLF+dLB+dRB+dRF)/4 前向、dvx=((dLF+dRB)-(dLB+dRF))/4 横向
  *        2) 旋转用统一标准矩阵（等价旧代码分象限4段if）：θ=(360-yaw)°，HWT101CT yaw 顺时针为正 → 镜像成逆时针
  *        3) 修复旧代码 now_v_x=delta_x/0.005 的分母 bug → 控制周期 ENCODER_TIME_S(0.01)
  */
void CHASSIS_Odom_Calculate(const int16_t pulse[5]){
  /* 4轮本周期位移 cm */
  float dist[5];
  for(uint8_t i = CHASSIS_MOTOR_LF; i <= CHASSIS_MOTOR_RF; i++){
    dist[i] = (pulse[i] / ENCODER_ACCURACY) * PERIMETER;
  }
  /* 车体坐标系位移：本车麦轮正解（4轮全部参与，与逆解自洽） */
  float dy_o = (dist[CHASSIS_MOTOR_LF] + dist[CHASSIS_MOTOR_LB] + dist[CHASSIS_MOTOR_RB] + dist[CHASSIS_MOTOR_RF]) / 4.0f;
  float dx_o = ((dist[CHASSIS_MOTOR_LF] + dist[CHASSIS_MOTOR_RB]) - (dist[CHASSIS_MOTOR_LB] + dist[CHASSIS_MOTOR_RF])) / 4.0f;
  /* 车头角 rad：yaw 0~360 顺时针为正 → (360-yaw) 为逆时针，归一化到 [0,2π) */
  const float pi = 3.14159265f;
  chassis.now_the = (360.0f - HWT101CT_Data.yaw) * pi / 180.0f;
  if(chassis.now_the >= 2.0f*pi)      chassis.now_the -= 2.0f*pi;
  else if(chassis.now_the < 0.0f)     chassis.now_the += 2.0f*pi;
  /* 车体系 → 全局系：标准旋转矩阵 */
  float dx = dx_o * cosf(chassis.now_the) - dy_o * sinf(chassis.now_the);
  float dy = dx_o * sinf(chassis.now_the) + dy_o * cosf(chassis.now_the);
  /* 速度(cm/s) + 位置(cm) 积分 */
  chassis.now_v_x = dx / ENCODER_TIME_S;
  chassis.now_v_y = dy / ENCODER_TIME_S;
  chassis.pos_x += dx;
  chassis.pos_y += dy;
}

/**
  * @brief 梯形速度规划启动（go_to_xy 移植，非阻塞）：x/y方向走固定距离 cm，自动停
  * @param x_dist  x方向移动距离 cm（>0 右移 / <0 左移 / 0 不移动）
  * @param y_dist  y方向移动距离 cm（>0 前进 / <0 后退 / 0 不移动）
  * @param x_speed x方向规划最大速度 cm/s
  * @param y_speed y方向规划最大速度 cm/s
  * @param x_acc   x方向规划加减速 cm/s²
  * @param y_acc   y方向规划加减速 cm/s²
  * @note  照搬旧代码 go_to_xy 前半：定方向 → calcTrapezoidalProfile 规划（起点静止、目标
  *        x_speed/y_speed、末速度0、加减速 x_acc/y_acc）→ ti=0 置规划标志立即返回。
  *        串口 mx/my 用 chassis.move_speed/move_acc 作默认速度/加速度；ROBOT_Move 传指定值。
  *        实际执行在 CHASSIS_Control_Loop 规划段：每周期 v_x/v_y=dir*calcTrapezoidalVel(tp,ti)，
  *        到 tp.t 清标志并归零 → 走完固定距离自动停（时间开环，无位置反馈）。
  *        规划期间角度环(flag.angle)保持锁向，保证走直线。
  */
void CHASSIS_Start_Move(float x_dist, float y_dist, float x_speed, float y_speed, float x_acc, float y_acc){
  /* 方向：正负决定移动方向，距离取绝对值；0 保持原方向（规划长度0自动停） */
  if(x_dist > 0.0f)      chassis.speed_dir_x =  1.0f;
  else if(x_dist < 0.0f) chassis.speed_dir_x = -1.0f;
  if(y_dist > 0.0f)      chassis.speed_dir_y =  1.0f;
  else if(y_dist < 0.0f) chassis.speed_dir_y = -1.0f;
  /* 梯形规划：起点静止、目标速度 x_speed/y_speed、末速度0、加减速 x_acc/y_acc */
  calcTrapezoidalProfile(fabsf(x_dist), 0.0f, x_speed, 0.0f, x_acc, x_acc, &chassis.tp_x);
  calcTrapezoidalProfile(fabsf(y_dist), 0.0f, y_speed, 0.0f, y_acc, y_acc, &chassis.tp_y);
  chassis.ti = 0.0f;
  chassis.x_speed_plan_flag = 1;   // 距离0的轴：tp.t=0，下周期立刻清标志（v_x 保持0）
  chassis.y_speed_plan_flag = 1;
  chassis.x_set_speed_flag  = 0;   // 规划期间由中断接管 v_x/v_y（手动设速让位）
  chassis.y_set_speed_flag  = 0;
}

/**
  * @brief 底盘10ms控制循环：麦轮解算 → 读编码器 → 速度换算(cm/s) → 增量式PID → PWM输出
  * @attention 在 TIM7 1ms 中断内每10ms调用一次；整车目标速度由上层赋值 chassis.v_x/v_y/w
  *           轮子编号即 TB6612/编码器/TIM1 通道编号，无需映射
  */
void CHASSIS_Control_Loop(void){
  /* 梯形速度规划执行段（go_to_xy 移植）：照搬旧代码 time_period_fun 规划逻辑（时间开环）
     ti 计时 → 到 tp.t 清规划标志 → 有规划按速度曲线算 v_x/v_y，无规划且非手动设速则归零
     x/y_set_speed_flag=1（串口 vx/vy 置位）时对应轴保持手动值，不受规划/归零影响 */
  if(chassis.x_speed_plan_flag || chassis.y_speed_plan_flag){
    chassis.ti += ENCODER_TIME_S;
    if(chassis.ti >= chassis.tp_x.t) chassis.x_speed_plan_flag = 0;   // x轴规划到时间 → 结束
    if(chassis.ti >= chassis.tp_y.t) chassis.y_speed_plan_flag = 0;   // y轴规划到时间 → 结束
  }
  if(chassis.x_speed_plan_flag) chassis.v_x = chassis.speed_dir_x * calcTrapezoidalVel(&chassis.tp_x, chassis.ti);
  else if(!chassis.x_set_speed_flag) chassis.v_x = 0.0f;              // 规划结束/无规划 → 归零停
  if(chassis.y_speed_plan_flag) chassis.v_y = chassis.speed_dir_y * calcTrapezoidalVel(&chassis.tp_y, chassis.ti);
  else if(!chassis.y_set_speed_flag) chassis.v_y = 0.0f;

  /* 航向环（角度环）：flag.angle=1 时角度环输出 w 接管整车角速度（默认开启）
     target_yaw 哨兵 → 首次进入锁定当前陀螺仪朝向（上电自动锁向） */
  if(flag.angle){
    if(chassis.target_yaw < 0.0f){                 // YAW_TARGET_NONE 哨兵（合法目标角度 0~360 ≥0）
      chassis.target_yaw = HWT101CT_Data.yaw;      // 锁定当前朝向
    }
    chassis.yaw = HWT101CT_Data.yaw;
    chassis.yaw_pid.target = chassis.target_yaw;
    /* 角度环 kp 按车速分段：合速度≤40cm/s 用 -0.019（低速走直线纠偏更柔、防来回猛纠），高速用默认 -0.028
       判速用整车目标速度 v_x/v_y（与速度环按 target 分段一致；静止 v≈0 落低速段，转向也因此更稳不过冲） */
    float move_spd = sqrtf(chassis.v_x * chassis.v_x + chassis.v_y * chassis.v_y);
    chassis.yaw_pid.kp = (move_spd <= YAW_KP_LOW_SPEED_BOUND) ? YAW_PID_KP_LOW : YAW_PID_KP;
    PID_Angle(&chassis.yaw_pid);                   // 输出 w（rad/s，逆时针为正）；error0 已做最短路径归一化(±180)
    /* 死区：误差进入死区 → 角度环w=0（纯旋转时清速度环彻底停转）
       两套死区按是否平移自动切换：静止旋转(无平移)用 TURN=1.0° 防静摩擦来回飘；
       走直线(有平移)用 MOVE=0.3°，让1°内偏航也被纠正，直线更直
       死区内清零使速度环从0重新累加；仅适用于纯旋转(无平移)场景，边转边走需另做 */
    float yaw_dz = (fabs(chassis.v_x) < 0.001f && fabs(chassis.v_y) < 0.001f)
                 ? YAW_DEAD_ZONE_TURN : YAW_DEAD_ZONE_MOVE;
    if(fabs(chassis.yaw_pid.error0) < yaw_dz){
      chassis.w = 0.0f;
      /* 纯旋转(无平移速度)：彻底停转 + 清空速度环状态 + 跳过速度环
         防止增量式PWM在静止时累积、克服静摩擦猛动造成来回飘（已验证有效） */
      if(fabs(chassis.v_x) < 0.001f && fabs(chassis.v_y) < 0.001f){
        for(uint8_t i = CHASSIS_MOTOR_LF; i <= CHASSIS_MOTOR_RF; i++){
          PID_INC *pid = &chassis.speed_pid[i];
          pid->target    = 0.0f;
          pid->actual    = 0.0f;
          pid->out       = 0.0f;   // 当前PWM输出清零（增量式从0重新累加）
          pid->i_out     = 0.0f;   // 清空积分
          pid->p_out     = 0.0f;
          pid->d_out     = 0.0f;
          pid->err       = 0.0f;   // 清误差历史，避免残留差分项
          pid->last_err  = 0.0f;
          pid->prev_err  = 0.0f;
          TB6612_Control(i, 0);    // 直接断电停转
        }
        return;                    // 跳过本轮速度环
      }
      /* 有平移速度(vy/vx≠0)：w=0 停旋转，不清速度环，继续跑麦轮+速度环使 vx/vy 生效 */
    }else{
      chassis.w = chassis.yaw_pid.out;             // 角度环接管 w
    }
  }
  CHASSIS_Mecanum();  // 整车 v_x/v_y/w → 4轮目标速度（每周期先算再跑速度环）
  /* 读4轮编码器一次（清零法），供里程计与速度环共用，避免二次读取读到0 */
  int16_t pulse[5];
  for(uint8_t i = CHASSIS_MOTOR_LF; i <= CHASSIS_MOTOR_RF; i++){
    pulse[i] = ENCODER_GetPulse(i);                                         // 本周期脉冲，前进为正
  }
  /* 里程计：4轮脉冲 → 车体位移 → 全局坐标积分 */
  CHASSIS_Odom_Calculate(pulse);
  for(uint8_t i = CHASSIS_MOTOR_LF; i <= CHASSIS_MOTOR_RF; i++){
    PID_INC *pid = &chassis.speed_pid[i];
    if(abs(pulse[i]) < 25){//软件滤波：|脉冲|>=25 视为丢数/噪声，不更新 actual（保持上次值）
      pid->actual = (pulse[i] / ENCODER_ACCURACY) * PERIMETER / ENCODER_TIME_S;  // 脉冲→cm/s
    }
    /* 分段PID：按目标速度所在区间切换该段 kp/ki/kd（4轮 target 相同则共用同一套段参数） */
    uint8_t seg = 0;
    if(pid->target > SPEED_SEG_BOUND_1) seg = 1;
    if(pid->target > SPEED_SEG_BOUND_2) seg = 2;
    if(pid->target > SPEED_SEG_BOUND_3) seg = 3;
    pid->kp = chassis.speed_seg[seg].kp;
    pid->ki = chassis.speed_seg[seg].ki;
    pid->kd = chassis.speed_seg[seg].kd;
    PID_IncUpdate(pid);                                                     // 增量式 PID（照搬旧代码算法）
    TB6612_Control(i, (int16_t)pid->out);                                   // PWM ±1000 直接输出
  }
}