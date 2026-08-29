#include "robot.h"
#include "chassis.h"
#include "hwt101ct.h"
#include "circle.h"
#include "main.h"
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
  while(fabs(chassis.target_yaw-chassis.yaw_pid.actual) >= YAW_DEAD_ZONE_TURN){
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

/**
  * @brief 绕车头前方一点做圆周运动（阻塞式，车头始终面向圆心，车头距圆心距离固定）
  * @param radius  车中心到圆心的距离 cm（= 目标测距 + 传感器偏置8cm + 管半径4cm）
  * @param arc_deg 绕行弧角（°）：绕满该角度自动停（360 = 整圈）
  * @param speed   切向速度 cm/s（>0 逆时针 / <0 顺时针 / 0 不转）
  * @note  前置条件：底盘控制循环运行（flag.chassis=1）且陀螺仪在更新（flag.hwt101ct=1），
  *        否则直接返回。底层调用 CIRCLE_Run（三层闭环：径向距离环 PID + 航向同步环
  *        + 切向速度前馈），用 GY53_2 测距实时保半径、陀螺仪做航向同步，并带测距低通
  *        滤波与极值搜索漂移修正，绕满弧角自动停。调用前请确保车头已正对水管
  *        （测距读到的是 传感器→管壁 的距离，不是斜距）。
  */
void ROBOT_Circle(float radius, uint32_t arc_deg, float speed){
  if(!flag.chassis || !flag.hwt101ct) return;                 // 前置条件不满足：直接返回
  if(radius <= 12.0f || arc_deg == 0 || speed == 0.0f) return;// 非法参数（radius 须 > 传感器偏置+管半径）
  /* 车中心→圆心距离 radius → 目标测距 mm（传感器→管壁） */
  uint16_t d_target_mm = (uint16_t)((radius - CIRCLE_SENSOR_OFFSET_CM - CIRCLE_PIPE_RADIUS_CM) * 10.0f + 0.5f);
  float omega = fabsf(speed) / radius;                         // 切向速度 speed → 公转角速度 rad/s
  int8_t dir  = (speed > 0) ? 1 : -1;                          // speed>0 逆时针 / <0 顺时针
  CIRCLE_Run(GY53_2_GPIO_Port, GY53_2_Pin, d_target_mm, omega, arc_deg, dir);
}
