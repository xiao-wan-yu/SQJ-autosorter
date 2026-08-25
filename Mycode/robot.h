#ifndef __ROBOT_H
#define __ROBOT_H

#include <stdint.h>

/* 高层任务函数：把底盘封装成比赛任务可调用的"动作"（阻塞式，调用时暂停所在主循环直至动作完成）
   ============ 坐标约定 ============
   - 角度  ：HWT101CT 定义，上电为 0°，顺时针旋转，0~360°（360 等效 0°）
   - 位移  ：基于车头当前方向 —— 车身前方为 y 正、车身右方为 x 正（与底盘 v_x/v_y 定义一致） */

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

#endif /* __ROBOT_H */
