/**
  * @file   circle.c
  * @brief  定半径圆周运动（三层闭环：径向距离环 + 航向同步环 + 切向前馈）
  * @note   豆包方案实现：
  *           1) 径向距离环 PID：e_D = D_target - (d_filt+偏置+管半径) → v_y
  *           2) 航向同步：目标航向 = 起始yaw + 公转角度 + 漂移修正，角度环 → Δω
  *           3) 切向前馈：v_x = dir·ω·D_target
  *           4) 低通滤波 + 无效测距保护 + 极值搜索漂移修正
  */

#include "circle.h"
#include "chassis.h"
#include "hwt101ct.h"
#include "gy53.h"
#include "uart.h"
#include "serialplot.h"
#include <math.h>
#include <string.h>

/* ==================== 模块参数/状态 ==================== */
CIRCLE_Param circle_param = {
  .stage         = CIRCLE_STAGE_DEF,      // 调试阶段：0 纯开环 → 4 全开（串口 cstage 在线切换）
  .kp            = CIRCLE_DIST_KP_DEF,
  .ki            = CIRCLE_DIST_KI_DEF,
  .kd            = CIRCLE_DIST_KD_DEF,
  .int_max       = CIRCLE_DIST_INT_MAX_DEF,
  .vy_max        = CIRCLE_VY_MAX_DEF,
  .yaw_kp        = CIRCLE_YAW_KP_DEF,
  .w_max         = CIRCLE_W_MAX_DEF,
  .alpha         = CIRCLE_FILTER_ALPHA_DEF,
  .d_min         = CIRCLE_D_VALID_MIN_DEF,
  .d_max         = CIRCLE_D_VALID_MAX_DEF,
  .drift_enable  = 1,
  .drift_step    = CIRCLE_DRIFT_STEP_DEF,
  .drift_period_ms = CIRCLE_DRIFT_PERIOD_DEF,
  .print_period_ms = CIRCLE_PRINT_PERIOD_DEF,
};

CIRCLE_State circle;

/* 目标航向：起始 yaw + 公转期望转角 + 漂移修正，归一化到 [0,360) */
static float circle_target_yaw(void){
  /* 公转期望转角：dir 逆时针时车自身逆时针转 → HWT101CT yaw 递减 */
  float yaw_change = -(float)circle.dir * circle.alpha * 180.0f / 3.14159265f;
  float t = circle.yaw_start + yaw_change + circle.yaw_offset;
  while(t >= 360.0f) t -= 360.0f;
  while(t <   0.0f)  t += 360.0f;
  return t;
}

/**
  * @brief 单步控制（10ms 级，内部用 tick 差值算 dt）
  * @retval 0 继续 / 1 绕满弧角
  */
uint8_t CIRCLE_Step(GPIO_TypeDef *dist_gpio, uint16_t dist_pin){
  uint32_t tick_now = HAL_GetTick();
  float dt = (float)(tick_now - circle.tick_last) / 1000.0f;
  circle.tick_last = tick_now;
  if(dt <= 0.0f)      dt = 0.01f;   // 兜底（正常由 tick 差值给出）
  else if(dt > 0.05f) dt = 0.05f;   // 防首次大步进（GY53 阻塞/卡顿保护）

  /* ---- 1. 测距读取 + 一阶低通滤波 + 无效保护（stage>=1 才读测距） ----
     stage 0 纯开环不读测距（省 GY53 阻塞，先只验证绕圈方向/快慢）；
     无效（杂散/测空）时保持上次滤波值 → D_actual 不变 → 距离环 e_D 不变 → v_y≈0，
     车只走切向+角速度前馈，避免测空瞬间猛插/猛退 */
  if(circle_param.stage >= 1){
    uint16_t d_raw = GY53_GetDistance_PWM(dist_gpio, dist_pin);   // mm
    if(d_raw >= circle_param.d_min && d_raw <= circle_param.d_max){
      float d_cm = (float)d_raw / 10.0f;
      circle.d_filt = circle_param.alpha * circle.d_filt + (1.0f - circle_param.alpha) * d_cm;
    }
    circle.D_actual = circle.d_filt + CIRCLE_SENSOR_OFFSET_CM + CIRCLE_PIPE_RADIUS_CM;
  }else{
    circle.D_actual = circle.D_target;   // 纯开环：无测距反馈，半径按理论值
  }

  /* ---- 2. 径向距离环 PID → v_y（stage>=2 才启用；太远前进、太近后退） ----
     带宽要低（kp 小、滤波重），避免与角度环互相耦合振荡 */
  float v_y = 0.0f;
  if(circle_param.stage >= 2){
    circle.dist_pid.target = circle.D_target;
    circle.dist_pid.actual = circle.D_actual;
    circle.dist_pid.kp  = circle_param.kp;
    circle.dist_pid.ki  = circle_param.ki;
    circle.dist_pid.kd  = circle_param.kd;
    circle.dist_pid.integral_max = circle_param.int_max;
    circle.dist_pid.out_max =  circle_param.vy_max;
    circle.dist_pid.out_min = -circle_param.vy_max;
    PID_PosUpdate(&circle.dist_pid);
    v_y = circle.dist_pid.out;
  }

  /* ---- 3. 公转角度积分 + 切向速度前馈 → v_x（所有 stage 都走） ----
     v_x = dir·ω·D_target（逆时针：向右走；顺时针：向左走） */
  circle.alpha += (float)circle.dir * circle.omega * dt;
  float v_x = (float)circle.dir * circle.omega * circle.D_target;

  /* ---- 4. 航向同步环（stage>=3 才启用）：目标航向 → 角度环 → Δω；w = ω 前馈 + Δω ----
     误差做 ±180 归一化（HWT101CT yaw 0~360，绕圈会跨越 0/360 边界）；
     纯 P（ki=kd=0），kp 为负（yaw 顺时针正、w 逆时针正，镜像） */
  float w = (float)circle.dir * circle.omega;         // 开环角速度前馈（所有 stage 都走）
  if(circle_param.stage >= 3){
    float e_yaw = circle_target_yaw() - HWT101CT_Data.yaw;
    while(e_yaw >  180.0f) e_yaw -= 360.0f;
    while(e_yaw < -180.0f) e_yaw += 360.0f;
    circle.yaw_pid.error1 = circle.yaw_pid.error0;
    circle.yaw_pid.error0 = e_yaw;
    circle.yaw_pid.kp = circle_param.yaw_kp;
    float d_w = circle_param.yaw_kp * e_yaw;
    if(d_w >  circle_param.w_max) d_w =  circle_param.w_max;
    else if(d_w < -circle_param.w_max) d_w = -circle_param.w_max;
    w += d_w;
  }
  if(w >  circle_param.w_max) w =  circle_param.w_max;
  else if(w < -circle_param.w_max) w = -circle_param.w_max;

  /* ---- 5. 极值搜索法修正陀螺仪漂移（stage>=4 才启用） ----
     正对管壁时测距最小：每周期给 yaw_offset 一个微小步长，测距变小则保持方向、
     变大则反向（爬山法）。步长小(0.2°)、周期长(200ms)，带宽最低，不干扰角度/距离环 */
  if(circle_param.stage >= 4 && circle_param.drift_enable
     && (tick_now - circle.drift_last >= (uint32_t)circle_param.drift_period_ms)){
    circle.drift_last = tick_now;
    if(circle.d_filt >= circle.drift_ref_d) circle.drift_dir = -circle.drift_dir;  // 测距没变小 → 反向
    circle.yaw_offset += circle.drift_dir * circle_param.drift_step;
    circle.drift_ref_d = circle.d_filt;
  }

  /* ---- 6. 陀螺仪累积转角（判断绕行弧角，处理 0/360 跳变） ---- */
  float dy = HWT101CT_Data.yaw - circle.yaw_last;
  circle.yaw_last = HWT101CT_Data.yaw;
  if(dy >  180.0f) dy -= 360.0f;
  else if(dy < -180.0f) dy += 360.0f;
  circle.yaw_acc += dy;

  /* ---- 7. 下发整车速度（中断里每 10ms 由 CHASSIS_Control_Loop 执行） ---- */
  chassis.v_x = v_x;
  chassis.v_y = v_y;
  chassis.w   = w;

  /* ---- 8. 周期打印（SerialPlot 观察收敛，默认 200ms 一次，cprint 可调，0 关闭） ---- */
  if(circle_param.print_period_ms > 0
     && (tick_now - circle.print_last >= (uint32_t)circle_param.print_period_ms)){
    circle.print_last = tick_now;
    CIRCLE_PrintState();
  }

  /* ---- 9. 串口在线调参（绕圈期间实时改参数，指令如 "ckp f 0.6" / "cyawkp f -0.04"） ---- */
  if(UART1_RxFlag){
    UART1_RxFlag = 0;
    SERIALPLOT_ChangeParam((char *)UART1_RxBuf);
  }

  /* 绕满弧角 → 结束 */
  if(fabsf(circle.yaw_acc) >= 360.0f) return 1;   // 兜底：整圈保护（arc_deg 最大 360）
  return 0;
}

/**
  * @brief 串口打印一次当前状态（单行多通道，SerialPlot 可直接画图）
  *        d    滤波后测距 cm×10     D   实际轨迹半径 cm×10
  *        eD   半径误差 mm          vx/vy/w 当前整车速度
  *        yawT 目标航向 °×10        yaw 实际航向 °×10
  */
void CIRCLE_PrintState(void){
  UART1_Printf("%d %d %d %d %d %d %d %d\r\n",
               (int)(circle.d_filt * 10.0f),
               (int)(circle.D_actual * 10.0f),
               (int)((circle.D_target - circle.D_actual) * 10.0f),
               (int)chassis.v_x,
               (int)chassis.v_y,
               (int)(chassis.w * 100.0f),
               (int)(circle_target_yaw() * 10.0f),
               (int)(HWT101CT_Data.yaw * 10.0f));
}

/**
  * @brief 定半径圆周运动（阻塞式，绕满弧角自动停）
  */
void CIRCLE_Run(GPIO_TypeDef *dist_gpio, uint16_t dist_pin,
                uint16_t d_target_mm, float omega, uint32_t arc_deg, int8_t dir){
  /* ---- 前置条件与参数保护 ---- */
  if(!flag.chassis || !flag.hwt101ct) return;
  if(d_target_mm < 50 || d_target_mm > 400) return;  // 目标测距异常（<5cm 或 >40cm）
  if(omega <= 0.0f || arc_deg == 0) return;
  if(dir >= 0) circle.dir =  1;                      // 方向：默认逆时针
  else         circle.dir = -1;
  if(circle_param.stage < 0) circle_param.stage = 0; // 阶段保护
  if(circle_param.stage > 4) circle_param.stage = 4;
  UART1_Printf("circle start stage=%d\r\n", circle_param.stage);  // 提示当前调试阶段

  /* ---- 保存外部状态，接管 w（角度环让位） ---- */
  uint8_t angle_save = flag.angle;
  flag.angle = 0;
  chassis.x_speed_plan_flag = 0;                     // 清掉距离规划，立即切换圆周模式
  chassis.y_speed_plan_flag = 0;
  chassis.x_set_speed_flag  = 1;                     // 手动设速：控制循环不再归零 v_x/v_y
  chassis.y_set_speed_flag  = 1;

  /* ---- 初始化运行状态 ---- */
  circle.running    = 1;
  circle.D_target   = (float)d_target_mm / 10.0f + CIRCLE_SENSOR_OFFSET_CM + CIRCLE_PIPE_RADIUS_CM;
  circle.omega      = omega;
  circle.alpha      = 0.0f;
  circle.yaw_start  = HWT101CT_Data.yaw;
  circle.yaw_offset = 0.0f;
  circle.yaw_acc    = 0.0f;
  circle.yaw_last   = HWT101CT_Data.yaw;
  circle.drift_dir  = 1;
  circle.tick_last  = HAL_GetTick();
  circle.drift_last = HAL_GetTick();
  circle.print_last = HAL_GetTick();

  /* ---- 初始测距采样：多次取有效值均值作滤波初值，避免从 0 起步产生冲量 ----
     调用前车头应已正对水管；采样期间车静止。stage 0 纯开环不读测距，直接用目标值 */
  if(circle_param.stage >= 1){
    float d_sum = 0.0f; uint8_t d_cnt = 0;
    for(uint8_t i = 0; i < 8; i++){
      uint16_t dd = GY53_GetDistance_PWM(dist_gpio, dist_pin);
      if(dd >= circle_param.d_min && dd <= circle_param.d_max){
        d_sum += (float)dd / 10.0f; d_cnt++;
      }
      HAL_Delay(20);
    }
    circle.d_filt = (d_cnt ? d_sum / (float)d_cnt : (float)d_target_mm / 10.0f);
  }else{
    circle.d_filt = (float)d_target_mm / 10.0f;
  }
  circle.D_actual    = circle.d_filt + CIRCLE_SENSOR_OFFSET_CM + CIRCLE_PIPE_RADIUS_CM;
  circle.drift_ref_d = circle.d_filt;

  /* ---- 距离环/航向环 PID 清零 ---- */
  memset(&circle.dist_pid, 0, sizeof(circle.dist_pid));
  memset(&circle.yaw_pid,  0, sizeof(circle.yaw_pid));

  /* ---- 阻塞循环：绕满弧角自动停 ---- */
  circle.tick_last = HAL_GetTick();                // 对齐控制起点，避免首次步进 dt 包含初始采样延时
  float arc_target = (arc_deg > 360) ? 360.0f : (float)arc_deg;
  while(fabsf(circle.yaw_acc) < arc_target){
    if(CIRCLE_Step(dist_gpio, dist_pin)) break;
  }

  /* ---- 收尾：停车 + 恢复角度环（重新锁向当前朝向） ---- */
  chassis.v_x = 0.0f;
  chassis.v_y = 0.0f;
  chassis.w   = 0.0f;
  chassis.x_set_speed_flag = 0;
  chassis.y_set_speed_flag = 0;
  flag.angle = angle_save;
  if(flag.angle) chassis.target_yaw = YAW_TARGET_NONE;   // 哨兵：恢复后锁定当前朝向
  circle.running = 0;
}

