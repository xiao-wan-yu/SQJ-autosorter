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

/* 各轮正/反转启动 PWM 阈值（2026-09-04 用户悬空实测：电机静止→持续转动的临界 PWM）。
   落地值略高于悬空值，靠 start_margin(裕量) 兜底；若需 >2.0 才同步则落地补测直接改表 */
static const StartPwm_t START_PWM[5] = {
    {0, 0},       // 索引 0 不用
    {110, 130},   // 1 左前 LF（最难启动）
    {90, 130},    // 2 左后 LB
    {80, 110},    // 3 右后 RB
    {60, 70},     // 4 右前 RF（最好启动）
};

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
  /* 航向环三档参数初值（每档一套 kp/ki/kd/bias，彼此独立）。控制循环按工况自动选一档、
     整套写进 yaw_pid，所以 yaw_pid 那三个值这里只是打底；串口 ykp1/2/3、yki1/2/3、
     ykd1/2/3、ybias1/2/3 在线调（档号 1旋转 2低速 3高速），调好填回 chassis.h 的宏 */
  chassis.yaw_param[YAW_STAGE_TURN].kp      = YAW_TURN_KP;
  chassis.yaw_param[YAW_STAGE_TURN].ki      = YAW_TURN_KI;
  chassis.yaw_param[YAW_STAGE_TURN].kd      = YAW_TURN_KD;
  chassis.yaw_param[YAW_STAGE_TURN].bias    = YAW_TURN_BIAS;
  chassis.yaw_param[YAW_STAGE_MOVE_LO].kp   = YAW_MOVE_LO_KP;
  chassis.yaw_param[YAW_STAGE_MOVE_LO].ki   = YAW_MOVE_LO_KI;
  chassis.yaw_param[YAW_STAGE_MOVE_LO].kd   = YAW_MOVE_LO_KD;
  chassis.yaw_param[YAW_STAGE_MOVE_LO].bias = YAW_MOVE_LO_BIAS;
  chassis.yaw_param[YAW_STAGE_MOVE_HI].kp   = YAW_MOVE_HI_KP;
  chassis.yaw_param[YAW_STAGE_MOVE_HI].ki   = YAW_MOVE_HI_KI;
  chassis.yaw_param[YAW_STAGE_MOVE_HI].kd   = YAW_MOVE_HI_KD;
  chassis.yaw_param[YAW_STAGE_MOVE_HI].bias = YAW_MOVE_HI_BIAS;
  chassis.yaw_pid.kp          = YAW_TURN_KP;   // 打底值（每周期被控制循环按档覆盖）
  chassis.yaw_pid.ki          = YAW_TURN_KI;
  chassis.yaw_pid.kd          = YAW_TURN_KD;
  /* 航向环档位状态（控制循环每周期刷新，见 chassis.h 的 YAW_STAGE_*） */
  chassis.yaw_stage           = YAW_STAGE_TURN;
  chassis.yaw_gear            = 0;              // 档位编码上电=旋转档（见 chassis.h）
  chassis.yaw_stage_cnt       = 0;
  chassis.yaw_stage_dz_ms     = 0;
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

<<<<<<< HEAD
  /* ==================== 每轮速度环 PID 参数表（★ 要调参就只改这张表，其余代码一律不用动） ====================
     行 = 轮号（1左前 2左后 3右后 4右前，第0行占位不用），列 = 速度段（控制循环按 |target| 选段，倒车同样适用）：
       段0: 0-40 cm/s    段1: 40-80    段2: 80-120    段3: 120-160
     每格三个数依次是 kp / ki / kd。
     ★ 当前状态（2026-09-18 落地实测）：**分两组**
         · 1 号(左前) / 4 号(右前)：kp=5.5 / ki=0.05 / kd=0
         · 2 号(左后) / 3 号(右后)：kp=4.4 / ki=0.04 / kd=0
       两组都填满各自的 4 个速度段（仍不分速度段）。1/4 号轮响应比 2/3 号慢，
       落地实测单独加大 kp/ki 后四轮一致性明显改善。
       起点来源：先落地带载实测出四轮统一值 4.4/0.04（明显优于之前悬空调出的 2.4/0.034
       —— 落地静摩擦远大于悬空，悬空那组偏软、推不动），再用串口单轮指令
       （kp1/ki1/kd1 … kp4/ki4/kd4，见 serialplot.c）单独修 1/4 号轮。
     结构保留的意义：以后若某轮或某速度段要单独定参（比如低速段起转差异大），
       直接改那一格即可，控制循环一行都不用动。
     背景：四轮静摩擦差异确实存在（悬空实测起转 PWM：右前60/70、右后80/110、左后90/130、
       左前110/130，最大差近 2 倍），故 <5cm/s 的低速段仍可能表现为先后起转，目前靠积分项爬升覆盖。 */
  static const SpeedSegParam SPEED_PID_DFT[5][SPEED_SEG_NUM] = {
    {{0.0f, 0.000f, 0.0f}, {0.0f, 0.000f, 0.0f}, {0.0f, 0.000f, 0.0f}, {0.0f, 0.000f, 0.0f}}, // 0 占位(编号从1起)
    {{5.5f, 0.05f, 0.0f}, {5.5f, 0.05f, 0.0f}, {5.5f, 0.05f, 0.0f}, {5.5f, 0.05f, 0.0f}}, // 1 左前 LF
    {{4.4f, 0.04f, 0.0f}, {4.4f, 0.04f, 0.0f}, {4.4f, 0.04f, 0.0f}, {4.4f, 0.04f, 0.0f}}, // 2 左后 LB
    {{4.4f, 0.04f, 0.0f}, {4.4f, 0.04f, 0.0f}, {4.4f, 0.04f, 0.0f}, {4.4f, 0.04f, 0.0f}}, // 3 右后 RB
    {{5.5f, 0.05f, 0.0f}, {5.5f, 0.05f, 0.0f}, {5.5f, 0.05f, 0.0f}, {5.5f, 0.05f, 0.0f}}, // 4 右前 RF
  };
  for(uint8_t i = CHASSIS_MOTOR_LF; i <= CHASSIS_MOTOR_RF; i++){
    for(uint8_t s = 0; s < SPEED_SEG_NUM; s++){
      chassis.speed_seg[i][s] = SPEED_PID_DFT[i][s];
    }
  }

  /* 单轮速度环临时调参（调试用，见 serialplot.c 的 SERIALPLOT_SpeedTuneLoop）：
     起点借用表格里"左前轮-低速段"那组，仅作串口调参的初始值，之后由串口 tkp/tki/tkd 在线改 */
  chassis.speed_tune = 0;                                             // 默认关闭：为0时以下字段不影响任何逻辑
  chassis.tune_wheel = CHASSIS_MOTOR_LF;
  chassis.tune_kp    = SPEED_PID_DFT[CHASSIS_MOTOR_LF][0].kp;
  chassis.tune_ki    = SPEED_PID_DFT[CHASSIS_MOTOR_LF][0].ki;
  chassis.tune_kd    = SPEED_PID_DFT[CHASSIS_MOTOR_LF][0].kd;
  chassis.tune_pulse = 0;
=======
  /* 分段PID参数表（用户实测标定：不同目标速度区间用不同参数，控制循环按 target 选段应用） */
  chassis.speed_seg[0].kp = 0.8f;   chassis.speed_seg[0].ki = 0.025f; chassis.speed_seg[0].kd = 0.0f; // 0-40
  chassis.speed_seg[1].kp = 2.5f;   chassis.speed_seg[1].ki = 0.02f;  chassis.speed_seg[1].kd = 0.0f; // 40-80
  chassis.speed_seg[2].kp = 3.0f;   chassis.speed_seg[2].ki = 0.05f;  chassis.speed_seg[2].kd = 0.0f; // 80-120
  chassis.speed_seg[3].kp = 3.5f;   chassis.speed_seg[3].ki = 0.02f;  chassis.speed_seg[3].kd = 0.0f; // 120-160

  /* 启动阈值整形 + 到位判停：裕量取默认值，累计位移/目标距离清零 */
  chassis.start_margin = START_PWM_MARGIN_DFT;
  chassis.dist_acc_x   = 0.0f;
  chassis.dist_acc_y   = 0.0f;
  chassis.move_target_x = 0.0f;
  chassis.move_target_y = 0.0f;
  /* 到位判停提前量：断电自然滑停等效减速度取默认值 */
  chassis.brake_decel = BRAKE_DECEL_DFT;
  /* 精细/常规分档：默认短距≤20cm、低速≤15cm/s，档位在 Start_Move / 恒速时按目标重算 */
  chassis.fine_max_dist = FINE_MOVE_MAX_DIST_DFT;
  chassis.fine_max_spd  = FINE_MOVE_MAX_SPD_DFT;
  chassis.fine_move     = 0;
>>>>>>> 6344796db1a8293c4e5bf52a95ab00726e7d1086
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
  *        3) 修复旧代码 now_v_x=delta_x/0.005 的分母 bug → 控制周期 ENCODER_TIME_S(0.020，20ms)
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
  /* 到位判停数据源：车体系实际位移 fabs 累计（判断"走够目标距离"用累计弧长即可，
     不依赖 yaw 旋转；MoveSpeed/原地旋转时无判停引用，累加无害） */
  chassis.dist_acc_x += fabsf(dx_o);
  chassis.dist_acc_y += fabsf(dy_o);
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
  /* 到位判停（位置闭环）：清零本段累计位移、记录目标距离（|x/y_dist|，距离0的轴=0 不参与判停） */
  chassis.dist_acc_x   = 0.0f;
  chassis.dist_acc_y   = 0.0f;
  chassis.move_target_x = fabsf(x_dist);
  chassis.move_target_y = fabsf(y_dist);
  /* 精细/常规分档：两轴目标距离均≤短距阈值(sdist)→精细档（启 启动整形+提前量判停断电）；
     否则常规档（大距离/高速）纯时间开环到 tp.t 自然停，不做位置判停、不做启动整形，
     避免提前量判停在 v~100+ 时提前约 v²/2a=50cm 就断电长滑行 → 又斜又偏。
     注意：常规档提前量判停被控制循环跳过（见判停块 fine_move 门），此处只记档位。 */
  chassis.fine_move = (chassis.move_target_x <= chassis.fine_max_dist
                    && chassis.move_target_y <= chassis.fine_max_dist) ? 1 : 0;
}

/* 到位断电停转（判停触发时调用）：结束规划后清零4轮速度环输出/积分/历史误差并断电。
   断电后轮子仍会按机械阻力自然滑行一段（等效减速度≈chassis.brake_decel≈100cm/s²），
   因此调用点由判停块按"目标−当前速度²/(2·brake_decel)"提前触发，滑停正好落在目标上。 */
static void CHASSIS_Stop_Now(void){
  for(uint8_t i = CHASSIS_MOTOR_LF; i <= CHASSIS_MOTOR_RF; i++){
    PID_INC *pid = &chassis.speed_pid[i];
    pid->target    = 0.0f;
    pid->out       = 0.0f;
    pid->i_out     = 0.0f;
    pid->p_out     = 0.0f;
    pid->d_out     = 0.0f;
    pid->err       = 0.0f;
    pid->last_err  = 0.0f;
    pid->prev_err  = 0.0f;
    TB6612_Control(i, 0);          // 直接断电
  }
}

/**
  * @brief 速度环段（4轮）：读编码器 → 里程计 → 脉冲换算cm/s → 取PID参数 → 增量式PID → PWM输出
  * @note  从 CHASSIS_Control_Loop 尾部原样抽出，供"正常行驶"与"单轮调参"两条路径共用
  */
static void CHASSIS_SpeedLoop(void){
  /* 读4轮编码器一次（清零法），供里程计与速度环共用，避免二次读取读到0 */
  int16_t pulse[5];
  for(uint8_t i = CHASSIS_MOTOR_LF; i <= CHASSIS_MOTOR_RF; i++){
    pulse[i] = ENCODER_GetPulse(i);                                         // 本周期脉冲，前进为正
    if(chassis.speed_tune && i == chassis.tune_wheel) chassis.tune_pulse = pulse[i];  // 被调轮脉冲（串口看量化台阶）
  }
  /* 里程计：4轮脉冲 → 车体位移 → 全局坐标积分（调参模式只有一轮在转，算出来是错的，跳过） */
  if(!chassis.speed_tune) CHASSIS_Odom_Calculate(pulse);
  for(uint8_t i = CHASSIS_MOTOR_LF; i <= CHASSIS_MOTOR_RF; i++){
    PID_INC *pid = &chassis.speed_pid[i];
    if(chassis.speed_tune && i != chassis.tune_wheel){
      /* 单轮调参：其余三轮彻底不驱动 —— 不跑PID、不给PWM、不更新actual。
         不能只靠"target=0 ⇒ 误差自然为0"：编码器噪声会让它们的 out 慢慢累起来，
         虽然到不了起转PWM(60~110)不会转，但会有微弱通电抖动/发热 */
      pid->out = 0.0f;
      TB6612_Control(i, 0);
      continue;
    }
    /* 软件滤波/健全性检查：|脉冲|>=100 视为丢数/噪声，不更新 actual（保持上次值）
       100 脉冲对应 100/878*39.9/0.020 ≈ 227cm/s，远超最大车速 160cm/s（仅 70 脉冲/周期），
       正常行驶永不触发，只用于拦截物理上不可能出现的野值。
       ★ 阈值必须跟着"每周期脉冲数"走，即正比于 ENCODER_ACCURACY × ENCODER_TIME_S：
         该乘积最初(439×10ms)为 4.39，对应阈值 25；2026-09-18 改 4 倍频后为 8.78 → 50；
         再改 20ms 后为 17.56 → 100。三次变动的物理门限始终是 227cm/s。
         算错的后果：阈值小于最大脉冲数时条件转假 ⇒ actual 停止更新 ⇒ 增量式PID 的 err 恒定
         ⇒ out 一路累加到限幅 900 ⇒ 高速飞车（且低速测试完全看不出来）。
         校验方法：阈值对应速度 = 阈值/878*39.9/0.020，必须大于最大车速 160cm/s。
         即：阈值 > 160/39.9*878*0.020 ≈ 70。改分辨率或控制周期时照着这个算。 */
    if(abs(pulse[i]) < 100){
      pid->actual = (pulse[i] / ENCODER_ACCURACY) * PERIMETER / ENCODER_TIME_S;  // 脉冲→cm/s
    }
    /* ★ 静止零漂保护（2026-09-18 加）：target=0 且本周期一个脉冲都没有 ⇒ 该轮就该静止，直接清零断电。
       为什么必须清：增量式 PID 的 i_out 是"累加器"（i_out += ki*err），err=0 时它不会归零，
       只被"防积死"夹到 ±0.2；而 pid.c:77 是 out += p_out + i_out + d_out —— 把整个累加器加上去，
       于是只要 i_out≠0，out 就每周期漂移 i_out（最多 0.2）：50Hz 下每秒漂 10，
       几秒后越过起转 PWM(60~110) ⇒ 轮子"动一下 → err 变号 → out 往回漂 → 再动一下"反复，
       而全程 target 和 actual 都是 0（现象：PWM 缓慢单调变化，轮子周期性抽搐）。
       与航向环死区里那段清速度环是同一思路，但那段只在 flag.angle=1 且无平移时才跑，
       覆盖不到：①四轮联动测试模式（关航向环）②斜移(45°)时解算出 target=0 的那两个轮子。
       注：target=0 但车还在滑行(pulse≠0)时不触发 —— 那种情况要靠 PID 正常刹车，不能断驱动。 */
    if(pid->target == 0.0f && pulse[i] == 0){
      pid->actual = 0.0f;
      pid->out    = pid->i_out = pid->p_out = pid->d_out = 0.0f;
      pid->err    = pid->last_err = pid->prev_err = 0.0f;
      TB6612_Control(i, 0);
      continue;
    }
    if(chassis.speed_tune){
      /* 单轮调参模式：全轮、全速段统一用临时参数（不查表、不分段），
         便于在串口上先试出一组值，再自己填进 CHASSIS_Init 的 SPEED_PID_DFT 表 */
      pid->kp = chassis.tune_kp;
      pid->ki = chassis.tune_ki;
      pid->kd = chassis.tune_kd;
    }else{
      /* 速度环PID参数：先按目标速度“大小”选速度段，再按轮号取该轮自己的那组 kp/ki/kd
         （每轮一套参数：四轮静摩擦差异大，初值表见 CHASSIS_Init 的 SPEED_PID_DFT）
         注意用 fabsf 取绝对值选段：原代码直接用带符号 target 判断，target<0（倒车）时 4 个条件
         全部为假 → seg 恒为 0，导致所有后退动作都在用最弱的段0参数(kp0.8/ki0.025)，而正车
         100cm/s 却用 kp3.5 —— 倒车与正车手感严重不对称，已修正为按速度大小选段 */
      float   t_abs = fabsf(pid->target);
      uint8_t seg = 0;
      if(t_abs > SPEED_SEG_BOUND_1) seg = 1;
      if(t_abs > SPEED_SEG_BOUND_2) seg = 2;
      if(t_abs > SPEED_SEG_BOUND_3) seg = 3;
      pid->kp = chassis.speed_seg[i][seg].kp;    // i = 轮号(1~4)，seg = 速度段
      pid->ki = chassis.speed_seg[i][seg].ki;
      pid->kd = chassis.speed_seg[i][seg].kd;
    }
    PID_IncUpdate(pid);                                                     // 增量式 PID（照搬旧代码算法）
    TB6612_Control(i, (int16_t)pid->out);                                   // PWM ±1000 直接输出
  }
}

/**
  * @brief 底盘20ms控制循环：麦轮解算 → 读编码器 → 速度换算(cm/s) → 增量式PID → PWM输出
  * @attention 在 TIM7 1ms 中断内每20ms调用一次（main.c 计数 >=20）；整车目标速度由上层赋值 chassis.v_x/v_y/w
  *           轮子编号即 TB6612/编码器/TIM1 通道编号，无需映射
  */
void CHASSIS_Control_Loop(void){
  /* 单轮速度环调参模式（临时调试，见 serialplot.c 的 SERIALPLOT_SpeedTuneLoop）：
     只跑4轮速度环，旁路 梯形规划/航向环/麦轮解算 —— 麦轮解算每周期都会覆盖4轮 target，
     不旁路就没法单独调某一个轮子（4轮速度线性相关，数学上不存在只让一轮非零的 v_x/v_y/w） */
  if(chassis.speed_tune){
    CHASSIS_SpeedLoop();
    return;
  }
  /* 梯形速度规划执行段（go_to_xy 移植）：照搬旧代码 time_period_fun 规划逻辑（时间开环）
     ti 计时 → 到 tp.t 清规划标志 → 有规划按速度曲线算 v_x/v_y，无规划且非手动设速则归零
     x/y_set_speed_flag=1（串口 vx/vy 置位）时对应轴保持手动值，不受规划/归零影响 */
  if(chassis.x_speed_plan_flag || chassis.y_speed_plan_flag){
    chassis.ti += ENCODER_TIME_S;
    if(chassis.ti >= chassis.tp_x.t) chassis.x_speed_plan_flag = 0;   // x轴规划到时间 → 结束
    if(chassis.ti >= chassis.tp_y.t) chassis.y_speed_plan_flag = 0;   // y轴规划到时间 → 结束
    /* 到位判停（位置闭环，2026-09-04 新增）：目标距离>0 且车体系累计位移≥目标 → 提前清标志停车。
       dist_acc 由 CHASSIS_Odom_Calculate 本函数下方累加，此判停读到的是上一周期值（滞后10ms可忽略）；
       本块在下方 if(flag) v=dir*vel else v=0 之前 → 清标志当周期 v 即归零。
       ti>=tp.t 时间判停保留，作"全车卡死 dist_acc 不涨"的兜底 */
    /* 提前量判停（2026-09-05 分档后仅精细档短距启用）：目标轴还剩的距离 ≤ 当前速度的滑行距离
       v²/(2·brake_decel)（再加10ms判停滞后补偿）就触发——结束规划 + 清空4轮输出断电，让车自然
       滑停，落点正好在目标距离上。若判停在"距离刚好撞上目标"才触发，断电后还会再滑 v²/(2a)（实测
       三档 a≈100cm/s²），必过头。
       常规档（大距离/高速）不走本块：提前量 v²/2a 在 v≈100 时约 50cm，会在离目标半米就断电，
       靠 50cm 不受控滑行到位 → 又斜又偏（队友长距走斜的实测元凶）。常规档只用上方 ti>=tp.t 的
       纯时间开环梯形（末速0自然减速），回到分档前已验证的整图走法。 */
    if(chassis.fine_move){
      if(chassis.move_target_x > 0.1f){
        float nowv   = fabsf(chassis.now_v_x);   // 上一周期实测整车x速度（判停读到的位移同理滞后一周期）
        float glide  = nowv*nowv/(2.0f*chassis.brake_decel) + nowv*ENCODER_TIME_S;
        if(chassis.dist_acc_x >= chassis.move_target_x - glide){
          chassis.x_speed_plan_flag = 0;
          if(chassis.move_target_y <= 0.1f) CHASSIS_Stop_Now();  // x 单轴到位 → 清空断电（斜走两轴同走不刹）
        }
      }
      if(chassis.move_target_y > 0.1f){
        float nowv   = fabsf(chassis.now_v_y);
        float glide  = nowv*nowv/(2.0f*chassis.brake_decel) + nowv*ENCODER_TIME_S;
        if(chassis.dist_acc_y >= chassis.move_target_y - glide){
          chassis.y_speed_plan_flag = 0;
          if(chassis.move_target_x <= 0.1f) CHASSIS_Stop_Now();  // y 单轴到位 → 清空断电
        }
      }
    }
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
    /* "有没有平移"——旋转档选择和下面的死区切换共用同一套判据，避免两处定义跑偏 */
    uint8_t no_move = (fabsf(chassis.v_x) < 0.001f && fabsf(chassis.v_y) < 0.001f);
    /* 航向环三档（2026-09-19 改：kp/ki/kd/bias **每档一套、彼此独立**，改前只有 kp 分档）：
       ① 无平移 → 旋转档（原地转向/被推动后纠偏，要快，kp 该猛）
       ②③④ 有平移时按分量最大值分三态：低于 YAW_MOVE_MIN_SPEED 不介入 / 到
             YAW_KP_LOW_SPEED_BOUND 为止是低速档（纠偏更柔、防来回猛纠）/ 之上是高速档
       判速用整车目标速度 v_x/v_y（与速度环按 target 分段一致），取**两分量绝对值的较大者**
       max(|vx|,|vy|)，而不是合速度 sqrt(vx²+vy²)。★ 2026-09-19 用户定。
       理由：麦轮斜移时四轮的平移分量是 vy±vx —— 越接近 45°，必有一对轮子的分量越接近 0
       （与车速快慢无关）。合速度在斜移时是虚高的（它反映车位移动速度，不是轮子被推着转
       多快）：(15,15) 合速度 21.2 会判"介入"，可慢轮根本没转；(60,60) 合速度 84.9 会判
       "高速档不介入"，可正拖着一个卡住的轮子跑，必然走歪。取 max 分量两个方向都更准。
       （纯轴向平移时两者完全等价 —— main.c 里绝大多数调用都是纯轴。
         旧注释按当时 BOUND=40 举的"vx=vy=30 会被 max 误判进低速档"现已不成立：
         BOUND 现为 80，合速度 42.4 和 max 30 都落低速档。）
       三档的值取 chassis.yaw_param[档]（不是宏）—— 因为本循环每周期都会覆盖 yaw_pid 的
       kp/ki/kd，直接改 yaw_pid.* 会被立刻冲掉，所以要串口 ykp1/2/3 等改数组才调得动。
      ★ 2026-09-19 加"不介入"：分量最大值低于 YAW_MOVE_MIN_SPEED 的平移，角度环整个不参与 ——
        kp/ki/kd 当 0 送进 PID、bias 也不叠 ⇒ out 恒 0 ⇒ w 恒 0。理由见 chassis.h
        的 YAW_MOVE_MIN_SPEED。档位本身仍报 YAW_STAGE_MOVE_LO（只有三个档位，
        "介不介入"另用 yaw_on 表达，不新增枚举值）。 */
    uint8_t motion_stage;                          // 本周期用哪一档（下面判停止档时也要用）
    uint8_t yaw_on = 1;                            // 本周期角度环是否介入（0 ⇒ w 恒 0）
    if(no_move){
      motion_stage = YAW_STAGE_TURN;
    }else{
      float move_max = fmaxf(fabsf(chassis.v_x), fabsf(chassis.v_y));   // 两分量绝对值的较大者
      if(move_max < YAW_MOVE_MIN_SPEED){
        motion_stage = YAW_STAGE_MOVE_LO;          // 名义落到低速档，但参数不生效
        yaw_on       = 0;
      }else{
        motion_stage = (move_max <= YAW_KP_LOW_SPEED_BOUND) ? YAW_STAGE_MOVE_LO
                                                            : YAW_STAGE_MOVE_HI;
      }
    }
    /* 档位编码（2026-09-19 加，仅供串口观察，不参与控制）：0=旋转 1=不介入 2=低速 3=高速
       ★ 为什么另立一个字段而不是用 chassis.yaw_stage：那个档位在确认停稳时会被改写成
         YAW_STAGE_STOP(3)，与本编码的"3=高速"撞车；而且它表达的是"停没停稳"，不是"介不介入"。 */
    if(no_move)                                chassis.yaw_gear = 0;
    else if(!yaw_on)                           chassis.yaw_gear = 1;
    else if(motion_stage == YAW_STAGE_MOVE_HI) chassis.yaw_gear = 3;
    else                                       chassis.yaw_gear = 2;

    const YawStageParam *yp = &chassis.yaw_param[motion_stage];   // 本档整套参数
    chassis.yaw_pid.kp = yaw_on ? yp->kp : 0.0f;   // 不介入 → 三项全 0，PID 输出恒 0
    chassis.yaw_pid.ki = yaw_on ? yp->ki : 0.0f;
    chassis.yaw_pid.kd = yaw_on ? yp->kd : 0.0f;
    PID_Angle(&chassis.yaw_pid);                   // 输出 w（rad/s，逆时针为正）；error0 已做最短路径归一化(±180)
    /* 输出偏置（静摩擦补偿，每档一个，见 chassis.h）：误差只剩零点几度时 P 项输出的 w
       经麦轮解算只剩几 cm/s，推不动轮子 ⇒ 叠一个与误差同向的固定偏置，保证任何非零误差
       都有"推得动轮子"的最小输出。方向取 error0 符号：error0>0 表示要顺时针转，
       而 kp 为负 ⇒ 偏置取负，与 out 同向。加完再限幅，别让偏置把输出顶出 out_max */
    if(yaw_on && yp->bias > 0.0f && chassis.yaw_pid.error0 != 0.0f){
      chassis.yaw_pid.out += (chassis.yaw_pid.error0 > 0.0f) ? -yp->bias : yp->bias;
      if(chassis.yaw_pid.out > YAW_PID_OUT_MAX) chassis.yaw_pid.out = YAW_PID_OUT_MAX;
      if(chassis.yaw_pid.out < YAW_PID_OUT_MIN) chassis.yaw_pid.out = YAW_PID_OUT_MIN;
    }
    /* 死区：误差进入死区 → w 归零（判定"已到位"，ROBOT_Angle 靠它返回）
       两套死区按是否平移自动切换：静止旋转(无平移)用 YAW_DEAD_ZONE_TURN；
       走直线(有平移)用 YAW_DEAD_ZONE_MOVE（低速/高速平移共用这一套，不再细分）。
       ★ 具体数值一律以 chassis.h 的 YAW_DEAD_ZONE_* 宏为准，这里不写死 —— 死区是实测反复
         微调的（1.0→0.65→0.75→0.92），写在这里的副本必然过期。 */
    float yaw_dz = no_move ? YAW_DEAD_ZONE_TURN : YAW_DEAD_ZONE_MOVE;
    if(fabs(chassis.yaw_pid.error0) < yaw_dz){
      chassis.w = 0.0f;
<<<<<<< HEAD
      /* ★ 只把 w 归零，不清速度环、不断电、不跳过速度环（2026-09-18 改，原为"清速度环+断电+return"）：
         原来那套是开环断电，车靠惯性自由滑行，一旦转过头出了死区就再没人管 —— 角度永久偏掉。
         改成保持速度环闭环后：
           ① 惯性还在转时，速度环以 target=0 主动反接刹车，比自由滑行刹得快、刹得准；
           ② 转过头出死区 → 角度环立刻接管反向修正，不会"进了死区就撒手"；
           ③ 车真正停稳后由 CHASSIS_SpeedLoop 的"静止零漂保护"(target==0 && pulse==0)
              清零断电，防增量式 PWM 累积漂移 —— 它比原来那段覆盖得更全（还管四轮联动
              测试模式和斜移解算出 target=0 的轮子），所以不会把"来回飘"放回来。 */
    }else{
      chassis.w = chassis.yaw_pid.out;             // 角度环接管 w
    }
    /* ---- 航向环档位状态（2026-09-19 加，见 chassis.h 的 YAW_STAGE_*） ----
       "到位"和"停稳"是两回事：上面那个死区判定只说明 **这一瞬间** 的误差采样点在死区内，
       车可能还带着惯性在滑 —— 原 ROBOT_Angle 就是拿它当返回判据的，返回时车往往还在动。
       停止档把两个条件都要求上：误差在死区内 **且** 四轮速度都低于阈值，连续保持若干周期。
       ★ 每周期刷新、不锁存：车又动了（出死区 / 有轮子转起来）就自动清计数、退回运动档。
       ★ speed_pid[i].actual 此刻读到的还是上一周期的值（速度环在本函数末尾 CHASSIS_SpeedLoop
         里才跑）—— 差 20ms 对"停稳"判定无影响，换来的是不必把这段挪到函数外面去。 */
    uint8_t wheels_stopped = 1;
    for(uint8_t i = CHASSIS_MOTOR_LF; i <= CHASSIS_MOTOR_RF; i++){
      if(fabsf(chassis.speed_pid[i].actual) >= YAW_STOP_SPEED_TH){ wheels_stopped = 0; break; }
    }
    uint8_t in_dz = (fabsf(chassis.yaw_pid.error0) < yaw_dz);
    if(in_dz && wheels_stopped){
      if(chassis.yaw_stage_cnt < 250) chassis.yaw_stage_cnt++;
    }else{
      chassis.yaw_stage_cnt = 0;                   // 误差出死区 / 有轮子又动了 → 重新计数
    }
    /* 进死区后开始计时（出死区清零）：超时兜底 —— 在死区边缘来回蹭时"四轮都停"永远不成立
       （航向环微动 + 惯性反复越过死区），没有它 ROBOT_Angle 会死等。 */
    if(in_dz){
      if(chassis.yaw_stage_dz_ms < 60000) chassis.yaw_stage_dz_ms += (uint16_t)(ENCODER_TIME_S * 1000.0f);
    }else{
      chassis.yaw_stage_dz_ms = 0;
    }
    uint8_t stop_ok = (chassis.yaw_stage_cnt >= YAW_STOP_CONFIRM)
                   || (YAW_STOP_TIMEOUT_MS > 0 && chassis.yaw_stage_dz_ms >= YAW_STOP_TIMEOUT_MS);
    chassis.yaw_stage = stop_ok ? YAW_STAGE_STOP : motion_stage;
=======
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
        /* 读空4轮编码器并喂里程计（关键修复 2026-09-05）：
           断电滑行/静止漂移期间轮子还在转，若不读清，这些脉冲会积压在编码器计数器里，
           等下一次 Start_Move 首拍被当成"单周期跑了几cm"一次性读入 → now_v 爆成几百cm/s、
           到位判停提前量 glide 爆炸 → 车一启动就被误判"已到位"断电（"my 第二次不动"根因）。
           这里每10ms读清并如实累计进里程计：滑行位移进入 pos/disti（OLED 与尺一致），
           now_v 也随真实轮速衰减到0，编码器永远不积压。 */
        int16_t idle_pulse[5];
        for(uint8_t i = CHASSIS_MOTOR_LF; i <= CHASSIS_MOTOR_RF; i++){
          idle_pulse[i] = ENCODER_GetPulse(i);
        }
        CHASSIS_Odom_Calculate(idle_pulse);
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
  /* 精细档判定（控制启动整形是否启用）：
       规划中 → 短距 Start_Move(fine_move=1) 才整形；
       恒速手动设速 → 合速度≤lspd 的慢速才整形（同步破静摩擦）；
       常规档（大距离/高速）不做整形 → 纯速度环自爬，回归分档前"整图跑通"的旧走法 */
  uint8_t do_shape = 0;
  if(chassis.x_speed_plan_flag || chassis.y_speed_plan_flag){
    do_shape = chassis.fine_move;
  }else if(chassis.x_set_speed_flag || chassis.y_set_speed_flag){
    float mag = sqrtf(chassis.v_x*chassis.v_x + chassis.v_y*chassis.v_y);
    if(mag > 0.5f && mag <= chassis.fine_max_spd + 0.01f) do_shape = 1;
  }
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
    /* 启动阈值整形（2026-09-04 新增，2026-09-05 起仅精细档启用）：该轮被命令要动(target≠0)但实测
       速度还很小(轮子没转起来)、且 PID 自身输出不够 → 抬到"该轮自己的启动阈值×裕量"，让4轮同一命令
       时刻同步破静摩擦。轮子一旦转起来(|actual|≥START_STILL_SPD)立即交还 PID；增量式下一拍按误差把
       输出拉回，不会持续顶满。常规档（大距/高速）跳过：4轮各自阻力差只影响起步瞬态，高速长距有足够
       加速段让速度环自爬，整形顶高反而可能引入瞬时横向速度差 */
    if(do_shape){
      float mrg = chassis.start_margin;
      if(     pid->target >  0.5f && pid->actual <  START_STILL_SPD
          && pid->out < START_PWM[i].fwd * mrg){
          pid->out = START_PWM[i].fwd * mrg;
      }else if(pid->target < -0.5f && pid->actual > -START_STILL_SPD
          && pid->out > -START_PWM[i].rev * mrg){
          pid->out = -START_PWM[i].rev * mrg;
      }
    }
    TB6612_Control(i, (int16_t)pid->out);                                   // PWM ±1000 直接输出
>>>>>>> 6344796db1a8293c4e5bf52a95ab00726e7d1086
  }
  CHASSIS_Mecanum();      // 整车 v_x/v_y/w → 4轮目标速度（每周期先算再跑速度环）
  CHASSIS_SpeedLoop();    // 读编码器 + 里程计 + 4轮速度环 + PWM输出
}
