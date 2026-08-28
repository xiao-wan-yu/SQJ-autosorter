#ifndef __ROBOT_H
#define __ROBOT_H

#include <stdint.h>

/* 高层任务函数：把底盘封装成比赛任务可调用的"动作"（阻塞式，调用时暂停所在主循环直至动作完成）
   ============ 坐标约定 ============
   - 角度  ：HWT101CT 定义，上电为 0°，顺时针旋转，0~360°（360 等效 0°）
   - 位移  ：基于车头当前方向 —— 车身前方为 y 正、车身右方为 x 正（与底盘 v_x/v_y 定义一致） */

/* ==================== 圆周运动参数（ROBOT_Circle 调参） ==================== */
//这里的参数不是真的，调要在main.c里调，robot.c里只是定义了一个默认值
#define ROBOT_CIRCLE_KP_R  1.0f   // 径向纠偏增益（1/s）：距圆心偏差(cm) → v_y(cm/s)
#define ROBOT_CIRCLE_KP_H  2.0f   // 航向纠偏增益（1/s）：车头与"指向圆心方向"的夹角误差(rad) → w(rad/s)

/**
  * @brief 原地旋转到目标角度（阻塞式，一般静止时旋转）
  * @param target_angle 目标角度 0~360°（上电0°、顺时针；360 等效 0）
  */
void ROBOT_Angle(uint32_t target_angle);

/**
  * @brief 沿车头当前方向走固定距离（阻塞式，梯形加减速，走完自动停）
  * @param x_distance x方向距离 cm（>0 右移 / <0 左移 / 0 不移动）
  * @param y_distance y方向距离 cm（>0 前进 / <0 后退 / 0 不移动）
  * @param x_maxspeed x方向最大速度 cm/s
  * @param y_maxspeed y方向最大速度 cm/s
  * @param x_maxa     x方向加减速 cm/s²
  * @param y_maxa     y方向加减速 cm/s²
  */
void ROBOT_Move(int32_t x_distance, int32_t y_distance,
                int32_t x_maxspeed, int32_t y_maxspeed,
                int32_t x_maxa,     int32_t y_maxa);

/**
  * @brief 设置整车恒定速度（非阻塞，持续移动，直到再次改速或停止）
  * @param x_speed x方向速度 cm/s（>0 右移 / <0 左移 / 0 不移动）—— 车身右方为 x 正
  * @param y_speed y方向速度 cm/s（>0 前进 / <0 后退 / 0 不移动）—— 车身前方为 y 正
  */
void ROBOT_MoveSpeed(float x_speed, float y_speed);

/**
  * @brief 绕车头前方一点做圆周运动（阻塞式，车头始终面向圆心，车头距圆心距离固定）
  * @param radius  车中心到圆心的距离 cm（圆心 = 调用时刻车头方向延长 radius 处的点，之后在场地中固定不动）
  * @param arc_deg 绕行弧角（°）：绕满该角度自动停（360 = 整圈）
  * @param speed   切向速度 cm/s（>0 逆时针 / <0 顺时针 / 0 不转）
  */
void ROBOT_Circle(float radius, uint32_t arc_deg, float speed);

#endif /* __ROBOT_H */
