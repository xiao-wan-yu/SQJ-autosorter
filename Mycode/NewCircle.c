/**
  * @file   NewCircle.c
  * @brief  新圆周运动：名义圆周运动（切向+角速度前馈）+ 激光左右微调 + 测距前后微调
  * @note   与 circle.c（旧方案：径向距离环 PID + 航向同步环 + 切向前馈）并列存在，
  *         旧方案保留不动，本文件是 2026-09 加了两个激光之后的新方案。
  *
  *         ┌─ 名义圆周运动（每个周期都给） v_x = dir·ω·D、w = dir·ω
  *         车 ─┼─ 激光 LASER3/LASER4 → 左右微调 v_x（脉冲式，±5cm/s）
  *         └─ 测距 GY53_2        → 前后微调 v_y（脉冲式，±5cm/s；两个激光都有障碍物才可信）
  *
  *         微调做法照搬阶梯阶段的思路：先速度置零/换向，再"给一小段速度 → 立刻判 → 再给"，
  *         单次修正都有最长时长上限，绝不让修正把车顶到水管上或把圆周运动停掉。
  *
  *         详细参数/调参说明/分步调试：见 Mycode/NewCircle.h
  */

#include "NewCircle.h"
#include "main.h"        /* GY53_2 / LASER3 / LASER4 的端口+引脚宏 */
#include "chassis.h"
#include "hwt101ct.h"
#include "gy53.h"
#include "laser.h"
#include "uart.h"
#include "serialplot.h"
#include <math.h>

/* ==================== 现场接线（换板子/换接口只改这 3 组） ====================
   · 测距：装车头正前方（镜头到车中心 NC_SENSOR_OFFSET_CM=14cm），读"镜头→水管表面"
   · 激光3：车头左前，装成"对着水管能反光"的角度（现场电位器已调好）
   · 激光4：车头右前，同上
   逻辑约定（与 NewCircle.h 头部一致）：两个都有障碍物=正对水管；
   只有3没障碍物=车偏右→往左补；只有4没障碍物=车偏左→往右补 */
#define NC_DIST_GPIO_PORT     GY53_2_GPIO_Port
#define NC_DIST_PIN           GY53_2_Pin
#define NC_LASER3_GPIO_PORT   LASER3_GPIO_Port
#define NC_LASER3_PIN         LASER3_Pin
#define NC_LASER4_GPIO_PORT   LASER4_GPIO_Port
#define NC_LASER4_PIN         LASER4_Pin

/* 常量：±180 归一化用的 360、弧度→角度 */
#define NC_RAD2DEG            57.29578f

/* ==================== 模块参数/状态 ==================== */
NCIRCLE_Param nc_param = {
  .stage         = NC_STAGE_DEF,
  .lr_speed      = NC_LR_SPEED_CM,
  .lr_chk_ms     = NC_LR_CHK_MS,
  .lr_ms         = NC_LR_MS,
  .fb_speed      = NC_FB_SPEED_CM,
  .fb_target_mm  = NC_D_TARGET_MM_DEF,
  .fb_tol_mm     = NC_FB_TOL_MM,
  .fb_trig_mm    = NC_FB_TRIG_MM,
  .fb_chk_ms     = NC_FB_CHK_MS,
  .fb_ms         = NC_FB_MS,
  .dist_read_ms  = NC_DIST_READ_MS,
  .alpha         = NC_ALPHA_DEF,
  .d_min         = NC_D_VALID_MIN_MM,
  .d_max         = NC_D_VALID_MAX_MM,
  .yaw_kp        = NC_YAW_KP_DEF,
  .w_max         = NC_W_MAX_DEF,
  .lost_ms       = (int)NC_LOST_MS,
  .timeout_ms    = (int)NC_TIMEOUT_MS,
  .print_ms      = NC_PRINT_MS,
};

NCIRCLE_State nc;

/* ==================== 小工具 ==================== */

/* 角度归一化到 [0,360) */
static float NC_Norm360(float a){
  while(a >= 360.0f) a -= 360.0f;
  while(a <    0.0f) a += 360.0f;
  return a;
}

/* 角度误差归一化到 (-180,180]：绕圈会跨 0/360 边界，必须先归一化再比较 */
static float NC_Norm180(float e){
  while(e >  180.0f) e -= 360.0f;
  while(e < -180.0f) e += 360.0f;
  return e;
}

/* 左右补方向 → 速度修正：lr_dir=-1 往左(v_x<0) / +1 往右(v_x>0) */
static float NC_LrFixOut(uint32_t now){
  nc.lr_out_dir = 0;
  if(nc_param.stage < 1 || nc.lr_dir == 0) return 0.0f;
  /* 单次左右微调最长 lr_ms：到点先停，等下一次采激光再决定（脉冲式，防补过头） */
  if((now - nc.lr_t0) >= (uint32_t)nc_param.lr_ms) return 0.0f;
  nc.lr_out_dir = nc.lr_dir;
  return (float)nc.lr_dir * nc_param.lr_speed;
}

/* 前后补方向 → 速度修正：fb_dir=+1 前进(v_y>0) / -1 后退(v_y<0) */
static float NC_FbFixOut(uint32_t now){
  nc.fb_out_dir = 0;
  if(nc_param.stage < 2 || nc.fb_dir == 0) return 0.0f;
  /* 单次前后微调最长 fb_ms：到点先停，下一个决策周期再判 */
  if((now - nc.fb_t0) >= (uint32_t)nc_param.fb_ms) return 0.0f;
  nc.fb_out_dir = nc.fb_dir;
  return (float)nc.fb_dir * nc_param.fb_speed;
}

/* ==================== 状态打印 ==================== */
/**
  * @brief 串口打印一次状态（单行多通道，SerialPlot 可直接画图）
  *        通道：d(滤波测距 mm) D(实际半径 mm) Dt(目标半径 mm) vx vy w(×100)
  *              lr(左右补方向) fb(前后补方向) l3 l4(激光有无障碍物)
  */
void NCIRCLE_PrintState(void){
  UART1_Printf("%d %d %d %d %d %d %d %d %d %d\r\n",
               (int)(nc.d_filt * 10.0f),        /* d  滤波后测距 mm */
               (int)(nc.D_actual * 10.0f),      /* D  实测轨迹半径 mm（车中心→管心） */
               (int)(nc.D_target * 10.0f),      /* Dt 目标轨迹半径 mm */
               (int)chassis.v_x,                /* 切向(含左右微调) cm/s */
               (int)chassis.v_y,                /* 径向(前后微调) cm/s */
               (int)(chassis.w * 100.0f),       /* 角速度 rad/s ×100 */
               (int)nc.lr_out_dir,              /* 当前左右补方向：-1左 / +1右 / 0不补 */
               (int)nc.fb_out_dir,              /* 当前前后补方向：-1后退 / +1前进 / 0不补 */
               (int)nc.l3_hit,                  /* LASER3 有障碍物? */
               (int)nc.l4_hit);                 /* LASER4 有障碍物? */
}

/* ==================== 绕圈主流程 ==================== */
void NCIRCLE_Run(uint16_t d_target_mm, float omega, uint32_t arc_deg, int8_t dir){
  /* ---- 前置条件与参数保护 ---- */
  if(!flag.chassis) return;                        /* 底盘控制循环没跑：直接返回 */
  if(omega <= 0.0f || arc_deg == 0U) return;       /* 参数非法 */
  if(d_target_mm < 30U || d_target_mm > 500U) return; /* 目标测距异常（超量程会返回 2000） */
  if(nc_param.stage >= 3 && !flag.hwt101ct) return;   /* 航向同步环要陀螺仪数据，没有就别擅自跑 */
  if(nc_param.stage < 0) nc_param.stage = 0;          /* 阶段保护（在线调参可能填错） */
  if(nc_param.stage > 3) nc_param.stage = 3;

  /* ---- 入场参数 ---- */
  nc_param.fb_target_mm = (int)d_target_mm;           /* 前后微调的目标测距(mm) */
  nc.dir        = (dir >= 0) ? 1 : -1;                /* 方向：1 逆时针 / -1 顺时针 */
  nc.omega      = omega;
  nc.D_target   = (float)d_target_mm / 10.0f + NC_SENSOR_OFFSET_CM + NC_PIPE_RADIUS_CM;
  nc.arc_target = (arc_deg > 360U) ? 360.0f : (float)arc_deg;
  UART1_Printf("NC start stage=%d dT=%dmm D=%dcm wx100=%d dir=%d arc=%d\r\n",
               nc_param.stage, (int)d_target_mm, (int)(nc.D_target * 10.0f),
               (int)(nc.omega * 100.0f), (int)nc.dir, (int)nc.arc_target);

  /* ---- 接管整车速度：角度环让位（w 由本模块给），手动设速标志置 1（控制循环不再归零 v_） ---- */
  uint8_t angle_save = flag.angle;
  flag.angle = 0;
  chassis.x_speed_plan_flag = 0;      /* 清掉距离规划，立即切换圆周模式 */
  chassis.y_speed_plan_flag = 0;
  chassis.x_set_speed_flag  = 1;
  chassis.y_set_speed_flag  = 1;

  /* ---- 状态清零 ---- */
  nc.running    = 1;
  nc.alpha_orbit = 0.0f;
  nc.yaw_start  = HWT101CT_Data.yaw;
  nc.yaw_last   = HWT101CT_Data.yaw;
  nc.yaw_acc    = 0.0f;
  nc.yaw_tgt    = HWT101CT_Data.yaw;
  nc.lr_dir = 0; nc.lr_out_dir = 0;
  nc.fb_dir = 0; nc.fb_out_dir = 0;
  nc.l3_hit = 0; nc.l4_hit = 0; nc.d_valid = 0;
  nc.fb_err_mm = 0;
  nc.lr_cnt = 0; nc.fb_cnt = 0;
  nc.d_raw  = d_target_mm;
  nc.tick_last   = HAL_GetTick();
  nc.lr_chk_tick = 0;                 /* 置 0 → 第一次 Step 立刻采激光/读测距 */
  nc.fb_chk_tick = 0;
  nc.dist_read_tick = 0;
  nc.run_t0 = nc.lost_t0 = nc.print_tick = nc.tick_last;

  /* ---- 初始测距采样：车静止、车头已对着水管，多次取有效值均值给滤波器定初值 ----
     （滤波值从 0 起步会让第一次前后补猛冲一下；stage 0 纯开环不读传感器） */
  if(nc_param.stage >= 1){
    float sum = 0.0f; uint8_t cnt = 0;
    for(uint8_t i = 0; i < 8; i++){
      uint16_t raw = GY53_GetDistance_PWM(NC_DIST_GPIO_PORT, NC_DIST_PIN);
      nc.d_raw = raw;
      if(raw >= nc_param.d_min && raw <= nc_param.d_max){ sum += (float)raw / 10.0f; cnt++; }
      HAL_Delay(20);
    }
    nc.d_filt  = cnt ? (sum / (float)cnt) : ((float)d_target_mm / 10.0f);
    nc.d_valid = cnt ? 1U : 0U;
    /* 激光也先看一眼：进圈瞬间就知道要不要左右补（结果也会打在第一帧日志里） */
    nc.l3_hit = LASER_Barrier(NC_LASER3_GPIO_PORT, NC_LASER3_PIN);
    nc.l4_hit = LASER_Barrier(NC_LASER4_GPIO_PORT, NC_LASER4_PIN);
  }else{
    nc.d_filt = (float)d_target_mm / 10.0f;
  }
  nc.D_actual = nc.d_filt + NC_SENSOR_OFFSET_CM + NC_PIPE_RADIUS_CM;
  UART1_Printf("NC ready d=%dmm D=%dmm l3=%d l4=%d yaw=%d\r\n",
               (int)(nc.d_filt * 10.0f), (int)(nc.D_actual * 10.0f),
               (int)nc.l3_hit, (int)nc.l4_hit, (int)HWT101CT_Data.yaw);

  /* ---- 阻塞循环：绕满弧角 / 总超时 / 丢目标 都会自己停 ---- */
  nc.tick_last = HAL_GetTick();       /* 对齐控制起点：首次步进的 dt 不含上面的初始采样延时 */
  uint8_t reason = NC_RUN_CONTINUE;
  while(reason == NC_RUN_CONTINUE){
    reason = NCIRCLE_Step();
  }

  /* ---- 收尾：停车 + 恢复角度环（哨兵：恢复后锁定当前朝向） ---- */
  chassis.v_x = 0.0f;
  chassis.v_y = 0.0f;
  chassis.w   = 0.0f;
  chassis.x_set_speed_flag = 0;
  chassis.y_set_speed_flag = 0;
  flag.angle = angle_save;
  if(flag.angle) chassis.target_yaw = YAW_TARGET_NONE;
  nc.running = 0;
  UART1_Printf("NC done reason=%d yawacc=%d yaw=%d d=%dmm lrN=%d fbN=%d\r\n",
               (int)reason, (int)nc.yaw_acc, (int)HWT101CT_Data.yaw,
               (int)nc.d_raw, (int)nc.lr_cnt, (int)nc.fb_cnt);
}


/* ==================== 单步控制 ==================== */
/**
  * @brief 单步控制（内部按 tick 差值算 dt，无需外部延时）
  * @retval NC_RUN_CONTINUE / NC_DONE_ARC / NC_DONE_TIMEOUT / NC_DONE_LOST
  */
uint8_t NCIRCLE_Step(void){
  uint32_t now = HAL_GetTick();

  /* ---- 0. 节拍（dt 只给航向环的理论公转角积分用） ---- */
  float dt = (float)(now - nc.tick_last) / 1000.0f;
  nc.tick_last = now;
  if(dt <= 0.0f)            dt = 0.01f;             /* 兜底 */
  else if(dt > NC_DT_MAX_S) dt = NC_DT_MAX_S;       /* 防首次/卡顿后步进过大 */

  /* ---- 1. 采激光 → 决定"左右"往哪补（stage>=1；每 lr_chk_ms 采一次） ----
     · 两个都有障碍物       → 车头正对水管、左右已对准 → 不用补
     · 只有 LASER3 没障碍物 → 车偏右 → 往左补（v_x 为负）
     · 只有 LASER4 没障碍物 → 车偏左 → 往右补（v_x 为正）
     · 两个都没有障碍物     → 偏太多(或车头歪太多)：NC_LR_BOTH_MISS_KEEP=1 按上次方向继续补，
                              置 0 则干脆不补 */
  if(nc_param.stage >= 1 && (now - nc.lr_chk_tick) >= (uint32_t)nc_param.lr_chk_ms){
    nc.lr_chk_tick = now;
    nc.l3_hit = LASER_Barrier(NC_LASER3_GPIO_PORT, NC_LASER3_PIN);
    nc.l4_hit = LASER_Barrier(NC_LASER4_GPIO_PORT, NC_LASER4_PIN);
    if(nc.l3_hit && nc.l4_hit)          nc.lr_dir = 0;                                 /* 两边都有：不补 */
    else if(!nc.l3_hit && nc.l4_hit)    nc.lr_dir = (int8_t)(-1 * NC_LR_FIX_SIGN);     /* 车偏右→往左补 */
    else if(nc.l3_hit && !nc.l4_hit)    nc.lr_dir = (int8_t)(+1 * NC_LR_FIX_SIGN);     /* 车偏左→往右补 */
    else if(!NC_LR_BOTH_MISS_KEEP)      nc.lr_dir = 0;                                 /* 都看不到：不补 */
    /* 两个都看不到且允许"保持"时：lr_dir 原样不动，继续按上次方向补 */
    nc.lr_t0 = now;                     /* 新一次左右微调的计时起点（最长 lr_ms） */
    if(nc.lr_dir != 0) nc.lr_cnt++;
  }
  uint8_t both_hit = (uint8_t)(nc.l3_hit && nc.l4_hit);   /* 两个激光都有障碍物（测距可信的前提） */
  if(nc.l3_hit || nc.l4_hit) nc.lost_t0 = now;            /* 任一激光看到水管 → 刷新"没丢目标"时刻 */

  /* ---- 2. 读测距 + 一阶低通滤波（stage>=1；dist_read_ms 节流，GY53 是阻塞读） ----
     无效读数(杂散/测空，超量程返回 2000)一律丢弃并保持上次滤波值 →
     前后补不会因为一次坏读数猛插/猛退 */
  if(nc_param.stage >= 1 && (now - nc.dist_read_tick) >= (uint32_t)nc_param.dist_read_ms){
    nc.dist_read_tick = now;
    uint16_t raw = GY53_GetDistance_PWM(NC_DIST_GPIO_PORT, NC_DIST_PIN);   /* mm */
    nc.d_raw = raw;
    if(raw >= nc_param.d_min && raw <= nc_param.d_max){
      nc.d_valid = 1;
      nc.d_filt  = nc_param.alpha * nc.d_filt + (1.0f - nc_param.alpha) * ((float)raw / 10.0f);
    }else{
      nc.d_valid = 0;
    }
  }
  nc.D_actual = nc.d_filt + NC_SENSOR_OFFSET_CM + NC_PIPE_RADIUS_CM;

  /* ---- 3. 用测距决定"前后"往哪补（stage>=2；每 fb_chk_ms 重判一次） ----
     ★只有两个激光都有障碍物时，测距才是正对水管的垂直距离（斜着测会偏大），默认才用它
     偏差 err>0：离太远 → 前进(v_y>0)；err<0：离太近 → 后退(v_y<0)；
     容差~触发之间保持上次方向（滞回），避免临界值附近来回微动 */
  if(nc_param.stage >= 2 && (now - nc.fb_chk_tick) >= (uint32_t)nc_param.fb_chk_ms){
    nc.fb_chk_tick = now;
    nc.fb_err_mm   = (int16_t)((int)(nc.d_filt * 10.0f + 0.5f) - nc_param.fb_target_mm);
    uint8_t trust  = (NC_FB_DIST_TRUST ? both_hit : 1);                        /* 测距可不可信 */
    int16_t aerr   = (nc.fb_err_mm < 0) ? (int16_t)(-nc.fb_err_mm) : nc.fb_err_mm;
    if(!nc.d_valid || !trust){                        /* 测距无效 / 左右没对准(测距不可信) → 不补 */
      nc.fb_dir = 0;
    }else if(aerr <= (int16_t)nc_param.fb_tol_mm){    /* 已在 目标±容差 内 → 不补 */
      nc.fb_dir = 0;
    }else if(aerr >= (int16_t)nc_param.fb_trig_mm){   /* 偏差够大 → 动（含换方向） */
      nc.fb_dir = (nc.fb_err_mm > 0) ? 1 : -1;
      nc.fb_cnt++;
    }
    /* tol~trig 之间：fb_dir 保持上次决定（滞回，不动） */
    nc.fb_t0 = now;                                   /* 新一次前后微调的计时起点（最长 fb_ms） */
  }

  /* ---- 4. 名义圆周运动 + 两个微调量合成 ----
     v_x = dir·ω·D_target（切向，保证车一直在转圈，任何情况下都不停）+ 左右微调
     v_y = 前后微调（径向：离太远前进 / 离太近后退）
     w   = dir·ω（公转角速度前馈，车头自己就会一直对着水管转） */
  float lr_fix = NC_LrFixOut(now);
  float fb_fix = NC_FbFixOut(now);
  float v_x = (float)nc.dir * nc.omega * nc.D_target + lr_fix;
  float v_y = fb_fix;
  float w   = (float)nc.dir * nc.omega;
  nc.alpha_orbit += (float)nc.dir * nc.omega * dt;          /* 理论公转角（航向环用） */

  /* ---- 5. 航向同步环（stage>=3）：目标航向 = 起始 yaw + 公转角度，让车头始终对准水管 ----
     纯 P、kp 为负（yaw 顺时针为正、w 逆时针为正，镜像）；w = 前馈 + Δw
     注意：目标角先归一化到 [0,360)，误差再归一化到 (-180,180] 再乘 kp，否则跨 0/360 会猛打 */
  if(nc_param.stage >= 3){
    float tgt = NC_Norm360(nc.yaw_start - (float)nc.dir * nc.alpha_orbit * NC_RAD2DEG);
    float e   = NC_Norm180(tgt - HWT101CT_Data.yaw);
    float d_w = nc_param.yaw_kp * e;
    if(d_w >  nc_param.w_max)      d_w =  nc_param.w_max;
    else if(d_w < -nc_param.w_max) d_w = -nc_param.w_max;
    w += d_w;
    nc.yaw_tgt = tgt;
  }
  if(w >  nc_param.w_max)      w =  nc_param.w_max;
  else if(w < -nc_param.w_max) w = -nc_param.w_max;

  /* ---- 6. 下发整车速度（10ms 中断里由 CHASSIS_Control_Loop 执行） ---- */
  chassis.v_x = v_x;
  chassis.v_y = v_y;
  chassis.w   = w;

  /* ---- 7. 陀螺仪累积转角：判断绕行弧角（处理 0/360 跳变） ---- */
  nc.yaw_acc += NC_Norm180(HWT101CT_Data.yaw - nc.yaw_last);
  nc.yaw_last = HWT101CT_Data.yaw;

  /* ---- 8. 周期打印（SerialPlot 看收敛；nprint 可调，0 关闭） ---- */
  if(nc_param.print_ms > 0 && (now - nc.print_tick) >= (uint32_t)nc_param.print_ms){
    nc.print_tick = now;
    NCIRCLE_PrintState();
  }

  /* ---- 9. 串口在线调参（绕圈期间实时改，如 "nstage i 1" / "nlrmv f 6" / "nfbtol i 8"） ---- */
  if(UART1_RxFlag){
    UART1_RxFlag = 0;
    SERIALPLOT_ChangeParam((char *)UART1_RxBuf);
  }

  /* ---- 10. 三种停车条件 ---- */
  if(fabsf(nc.yaw_acc) >= nc.arc_target) return NC_DONE_ARC;                 /* 绕满弧角：正常结束 */
  if(nc_param.timeout_ms > 0
     && (now - nc.run_t0) >= (uint32_t)nc_param.timeout_ms) return NC_DONE_TIMEOUT;
  if(nc_param.lost_ms > 0
     && (now - nc.lost_t0) >= (uint32_t)nc_param.lost_ms)   return NC_DONE_LOST;
  return NC_RUN_CONTINUE;
}
