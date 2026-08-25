#include "robot.h"
#include "chassis.h"
#include "stm32f4xx_hal.h"
#include <math.h>

/**
  * @brief 原地旋转到目标角度（阻塞式，一般静止时旋转）
  * @param target_angle 目标角度 0~360°（HWT101CT：上电为 0°，顺时针旋转对应 0~360°）
  * @note  前置条件：底盘控制循环运行（flag.chassis=1）且航向环开启（flag.angle=1），
  *        否则角度环不收敛，本函数直接返回。
  *        流程：设目标角度 → 阻塞等待航向环误差进入死区（静止旋转死区 YAW_DEAD_ZONE_TURN=1.0°）
  *        → 进入死区后再稳定 20ms（车已静止）才跳出。调用期间 10ms 中断里角度环照常纠偏。
  *        等待死区与控制循环的静止死区一致（静止旋转场景），无需在线调。
  */
void ROBOT_Angle(uint32_t target_angle){
  if(!flag.angle || !flag.chassis) return;     // 前置条件不满足：无法收敛，直接返回
  chassis.target_yaw = (float)target_angle;    // 设目标角度（0~360；360 由 PID_Angle 归一化等效 0）
  /* 阻塞等待误差进死区（error0 为最短路径误差，PID_Angle 已做 ±180 归一化） */
  while(fabs(chassis.yaw_pid.error0) >= YAW_DEAD_ZONE_TURN){
    HAL_Delay(5);
  }
  HAL_Delay(20);                               // 进入死区后再稳定 20ms 再跳出
}

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
  *        调用期间航向环（flag.angle=1 时）保持锁向走直线。规划结束 v_x/v_y 已归零，
  *        若需等车惯性完全停下，请在调用侧酌情再加延时。
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
    HAL_Delay(5);
  }
}
