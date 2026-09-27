#include "robot.h"
#include "chassis.h"
#include "hwt101ct.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "serialplot.h"   // SERIALPLOT_WheelActualPump()：下面几个阻塞等待循环里实时发四轮实际值
#include <math.h>

/* ==================== 平移期间的角度环行为（2026-09-19 重做） ====================
   ★ 曾有过一版"ROBOT_Move 进入就把三档参数全清零、ROBOT_Angle 再恢复"的强制屏蔽
     （yaw_param_save[3][4] + yaw_param_saved + ROBOT_YawParamBypass/Restore 两个 static
     函数），2026-09-19 按用户要求**整块删除**。
     删的原因：它让 ROBOT_Move 一律不纠偏 —— 无论走 11cm 还是 424cm，长距走直线全靠底盘
     自己。而这次要的是"按速度决定纠不纠"，一刀切的做法实现不了。
   现在改为**控制循环按合速度自动决定**，robot.c 不再插手（见 chassis.c 航向环段、
   chassis.h 的 YAW_MOVE_MIN_SPEED / YAW_KP_LOW_SPEED_BOUND）：
     ★ 2026-09-27 用户要求：0~20cm/s 的平移也介入、且用低速档那套参数 ⇒
       YAW_MOVE_MIN_SPEED 由 20 改 0，"不介入"区间并入低速档，实际只剩两档：
     max(|vx|,|vy|) ≤ 80cm/s → 低速平移档（kp -0.13 / kd 0.04 / bias 0.03），0~80 全覆盖
     > 80cm/s                 → 高速平移档（kp -0.035 / kd 0.025 / bias 0）
   （旧行为留档：max(|vx|,|vy|) < 20cm/s → 角度环不介入（w 恒 0）—— 把 chassis.h 的
     YAW_MOVE_MIN_SPEED 改回 20.0f 即恢复，代码不用动）
   ★ 对 ROBOT_Move 的影响：它跑的梯形规划有加速/匀速/减速段，速度不同 ⇒ 一次动作内部会
     自动切"低速档 ↔ 高速档"（旧版是"不介入 ↔ 低速档"）。**减速段末尾 v 归零那一刻落回
     旋转档**，若车头此时不在 target_yaw 上，角度环会立刻纠偏、把轮子转起来，紧随其后的
     等停循环可能被拖住（最多 MOVE_STOP_TIMEOUT_MS）。这是有意接受的代价，不是 bug。
   ★ ROBOT_MoveSpeed 同理；另注意 main.c 里十几处 ROBOT_MoveSpeed(10~20cm/s) 原来整个
     平移期间都不纠偏，现在整个行程（含起步/收尾的极低速段）都会被拉向 target_yaw。 */

/**
  * @brief 原地旋转到目标角度（阻塞式，一般静止时旋转）
  * @param target_angle 目标角度 0~360°（HWT101CT：上电为 0°，顺时针旋转对应 0~360°）
  * @note  前置条件：底盘控制循环运行（flag.chassis=1）且航向环开启（flag.angle=1），
  *        否则角度环不收敛，本函数直接返回。
  *        流程：设目标角度 → 阻塞等待航向环进入"停止档"（见 chassis.h 的 YAW_STAGE_*）。
  *        停止档要求 误差已在死区内 **且** 四轮速度都低于阈值，连续保持 YAW_STOP_CONFIRM
  *        个控制周期（默认 200ms）—— 由 20ms 控制循环每周期刷新，本函数只读不判。
  *        ★ 2026-09-19 改：原判据是"误差一进死区就返回"，但那只能保证某一瞬间误差 <1°，
  *          车可能还带着惯性在滑，返回时调用侧的下一条动作会叠在没停稳的车上。
  *          超时兜底见 YAW_STOP_TIMEOUT_MS（死区边缘来回蹭时"四轮都停"永远不成立）。
  *        调用期间控制循环里角度环照常纠偏。
  */
void ROBOT_Angle(uint32_t target_angle){
  if(!flag.angle || !flag.chassis) return;     // 前置条件不满足：无法收敛，直接返回
  chassis.target_yaw = (float)target_angle;    // 设目标角度（0~360；360 由 PID_Angle 归一化等效 0）
  /* 清掉上一次动作残留的停止档，并让"停稳确认"从零开始计数：档位是每周期刷新、不锁存的，
     不清的话若上一动作刚结束、车还稳稳停在目标角附近，本函数一进来就会被判成"已在停止档"
     直接返回，等于没转。 */
  chassis.yaw_stage       = YAW_STAGE_TURN;
  chassis.yaw_stage_cnt   = 0;
  chassis.yaw_stage_dz_ms = 0;
  while(chassis.yaw_stage != YAW_STAGE_STOP){
    /* 串口1实时发四轮实际值（2026-09-25 加，见 serialplot.c）：原地转向时四轮都在转，
       这里补上就有数据；发不发/隔多久由泵函数自己按 HAL_GetTick 节流，不阻塞本循环 */
    SERIALPLOT_WheelActualPump();
    HAL_Delay(5);
  }
}

/* ==================== ROBOT_Move 等停参数（2026-09-18 加） ====================
   用途：ROBOT_Move 返回前等四轮真正停稳。规划是纯时间开环，退出时只是"减速段走完、
   目标速度已归零"，车还带着惯性在滑 —— 此时立刻返回，调用侧的下一条动作就会叠在
   一台还没停稳的车上。*/
#define MOVE_STOP_SPEED_TH   3.0f   // 单轮算"停住"的速度阈值 cm/s（20ms 下 1 个编码器脉冲=2.27cm/s，取约 1 个台阶）
#define MOVE_STOP_CONFIRM    40     // 判据要连续满足多少次才算停稳（每次 HAL_Delay(5) ⇒ 约 200ms）
                                    // 2026-09-21 用户定 200ms（原 8 次 ≈ 40ms）：与 chassis.h 的
                                    // YAW_STOP_CONFIRM(10 个控制周期 = 200ms) 对齐，两条函数同长
                                    // ★ 每次迭代实际比 5ms 略多（还要跑 4 轮判断），所以是"≥200ms"
#define MOVE_STOP_TIMEOUT_MS 8000   // 等停超时 ms（坡道/外力都可能让轮子一直微动，到点必须放行）
                                    // 2026-09-21 用户定 8000ms（原 3000）
                                    // ★ 现在"航向环纠偏让轮子微动"是真的会发生了（见文件头）：
                                    //   规划跑完 v 归零 → 落旋转档 → 车头不在 target_yaw 上就纠偏
                                    //   → 轮子转起来，本循环的"四轮都停"判据被拖住，最坏到点放行

/**
  * @brief 沿车头当前方向走固定距离（阻塞式，梯形加减速，走完自动停）
  * @param x_distance x方向距离 cm（>0 右移 / <0 左移 / 0 不移动）—— 车身右方为 x 正
  * @param y_distance y方向距离 cm（>0 前进 / <0 后退 / 0 不移动）—— 车身前方为 y 正
  * @param x_maxspeed x方向规划最大速度 cm/s
  * @param y_maxspeed y方向规划最大速度 cm/s
  * @param x_maxa     x方向规划加减速 cm/s²
  * @param y_maxa     y方向规划加减速 cm/s²
  * @note  前置条件：底盘控制循环运行（flag.chassis=1），否则规划不执行，本函数直接返回。
  *        梯形速度规划（加速-匀速-减速），走完自动停（v_x/v_y 归零），阻塞等待规划结束。
  *        位移是车身坐标系（前方 y+、右方 x+），与底盘 v_x/v_y 定义一致，直接映射。
  *        ★ 调用期间角度环**不再被屏蔽**（原 bypass 机制已删，见文件头）：平移中按合速度
  *          自动选档纠偏 —— 0~80cm/s 走低速档、>80cm/s 走高速档（2026-09-27 起
  *          YAW_MOVE_MIN_SPEED=0，0~20cm/s 也纠偏；以前这一段不介入）。
  *        规划结束 v_x/v_y 已归零，随后再等"四轮目标速度为0 且 实测速度停稳 且 角度环已在
  *        死区内(w==0)"连续 200ms 保持才返回（最长 MOVE_STOP_TIMEOUT_MS=8000ms 超时放行），
  *        返回前清掉四轮速度环积分项。
  */
void ROBOT_Move(int32_t x_distance, int32_t y_distance,
                int32_t x_maxspeed, int32_t y_maxspeed,
                int32_t x_maxa,     int32_t y_maxa){
  if(!flag.chassis) return;                    // 前置条件不满足：规划不执行，直接返回
  CHASSIS_Start_Move((float)x_distance, (float)y_distance,
                     (float)x_maxspeed, (float)y_maxspeed,
                     (float)x_maxa,     (float)y_maxa);
  /* 阻塞等待规划结束（中断里到 tp.t 清标志；距离0的轴 tp.t=0 立即清） */
  while(chassis.x_speed_plan_flag || chassis.y_speed_plan_flag){
    SERIALPLOT_WheelActualPump();   // 串口1实时发四轮实际值（行进期间，见 serialplot.c）
    HAL_Delay(5);
  }

  /* 规划结束后再等车真正停稳（2026-09-18 加，2026-09-21 加严）：上一步跳出时只是"减速段走完
     + v_x/v_y 已归零"，车还带着惯性在滑，直接返回的话调用侧的下一条动作会叠在没停稳的车上。
     判据（三条全部满足，连续保持 MOVE_STOP_CONFIRM 次）：
       ① 四轮**目标速度**严格为 0   ② 四轮**实测速度**低于阈值   ③ 角度环已在死区内（w==0）
     ★ ①为何用严格 ==0：target 是控制循环上一周期刚算出来的（v_y±v_x∓w*half_sum），四项都归零
       就是精确的 0.0f，没有量化噪声。它非 0 说明"还有指令在推轮子"（典型：航向环在纠偏）。
     ★ ②为何只能用阈值：actual 来自编码器，20ms 下 1 个脉冲就是 2.27cm/s，车基本停住时
       偶尔抖出 1 个脉冲，actual 在 0 和 ±2.27 之间跳，严格判 0 会永远等不到。
     ★ ③为何判 chassis.w 而不是 yaw_pid.out：死区内控制循环把 w 归零，而 yaw_pid.out 那个
       PID 内部计算值并不清零，拿它判会永远不成立（见 chassis.c 航向环死区段）。
     ★ 必须有超时兜底：地面有坡、有人推车、或航向环纠偏让轮子微动时，判据永远不成立，
       到点必须放行，否则队友的比赛流程会卡死在这里。超时同样走下面的清积分。
     注：actual/target/w 由 TIM7 中断里的控制循环写、这里读；M4 上 float 单次访问不会撕裂，
        无需临界区。 */
  uint32_t stop_t0  = HAL_GetTick();
  uint8_t  stop_cnt = 0;
  while(HAL_GetTick() - stop_t0 < MOVE_STOP_TIMEOUT_MS){
    uint8_t stopped = 1;
    for(uint8_t i = CHASSIS_MOTOR_LF; i <= CHASSIS_MOTOR_RF; i++){
      if(chassis.speed_pid[i].target != 0.0f){ stopped = 0; break; }                       // ① 目标速度必须为 0
      if(fabsf(chassis.speed_pid[i].actual) >= MOVE_STOP_SPEED_TH){ stopped = 0; break; }  // ② 实测速度停住
    }
    if(stopped && chassis.w == 0.0f){                                                      // ③ 角度环已在死区
      if(++stop_cnt >= MOVE_STOP_CONFIRM) break;   // 连续保持够久 → 确认停稳
    }else{
      stop_cnt = 0;                                // 轮子又动 / 航向环又输出 → 重新计数
    }
    SERIALPLOT_WheelActualPump();                  // 串口1实时发四轮实际值（等停期间：正好看"反接刹车"收得多快）
    HAL_Delay(5);
  }
  /* 退出前清四轮速度环积分项（2026-09-21 用户要求）：这一趟规划攒下的 i_out 不该带到下一个
     动作去，否则下次起步会带着旧积分多冲一下。正常停稳和超时放行都清（两条路都走到这）。
     ★2026-09-27 位置式接入：位置式速度环的积分是 speed_pid_pos[i].errorint（累加量 Σerror，
       等价于增量式的 i_out），两套一起清；不清的话默认走位置式时这句等于没清、下次起步照样多冲。 */
  for(uint8_t i = CHASSIS_MOTOR_LF; i <= CHASSIS_MOTOR_RF; i++){
    chassis.speed_pid[i].i_out = 0.0f;
    chassis.speed_pid_pos[i].errorint = 0.0f;
  }
}

/**
  * @brief 设置整车恒定速度（非阻塞，持续移动，直到再次改速或停止），自动锁向走直线
  * @param x_speed x方向速度 cm/s（>0 右移 / <0 左移 / 0 不移动）—— 车身右方为 x 正
  * @param y_speed y方向速度 cm/s（>0 前进 / <0 后退 / 0 不移动）—— 车身前方为 y 正
  * @note  前置条件：底盘控制循环运行（flag.chassis=1），否则速度环不执行，本函数直接返回。
  *        调用后车持续以该速度移动（x/y_set_speed_flag=1 保持手动设速，控制循环不再归零），
  *        直到再次调用本函数改速，或调用侧清除标志并归零停止：
  *            chassis.x_set_speed_flag = 0;  chassis.y_set_speed_flag = 0;
  *            chassis.v_x = 0.0f;            chassis.v_y = 0.0f;
  *        自动开启航向环：target_yaw 置哨兵 → 控制循环首次进入即锁定调用时刻的当前朝向，
  *        恒速移动全程锁向走直线（不受此前 ROBOT_Angle 遗留目标角影响）。
  *        若调用前有距离规划在跑（ROBOT_Move 未结束），会先清掉规划标志，立即切换恒速模式。
  */
void ROBOT_MoveSpeed(float x_speed, float y_speed){
  if(!flag.chassis) return;                    // 前置条件不满足：速度环未运行，直接返回
  chassis.x_speed_plan_flag = 0;               // 清掉距离规划，立即切换恒速模式
  chassis.y_speed_plan_flag = 0;
  chassis.x_set_speed_flag  = 1;               // 手动设速标志：控制循环不再归零 v_x
  chassis.y_set_speed_flag  = 1;
  flag.angle                = 1;               // 开启航向环，移动时锁向走直线
  chassis.target_yaw        = YAW_TARGET_NONE; // 哨兵：锁定调用时刻的当前朝向
  chassis.v_x = x_speed;
  chassis.v_y = y_speed;
}

/* 绕圈（圆周/绕柱）：走 Core/Src/main.c 的 LiZhu_Circle_Run()（8.28"绕柱闭环"原版） */
