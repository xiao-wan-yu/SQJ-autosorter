#ifndef __QUATERNION_H
#define __QUATERNION_H

#include <stdint.h>

#define RAD_TO_DEG  57.295779513f
#define DEG_TO_RAD  0.017453292f

/* Mahony 算法参数 */
#define MAHONY_KP  0.5f      // 比例增益，控制加速度计修正速度（增大可改善Roll/Pitch漂移）
#define MAHONY_KI  0.002f    // 积分增益，控制陀螺仪零偏修正速度（保持较小值避免震荡）

/* 采样频率相关 */
#define SAMPLE_FREQ 200.0f   // 采样频率 Hz，根据实际 ODR 设置调整

/* ==================== 传感器轴配置 ====================
 * 根据传感器实际安装方向调整映射关系
 * 格式: AXIS_MAP(src_axis, sign)
 *   src_axis: 0=X, 1=Y, 2=Z
 *   sign:     1=正方向, -1=反方向
 *
 * 常见配置示例:
 * 1. 标准配置: 传感器正面朝上，X向前，Y向左，Z向上
 *    #define AXIS_MAP_X  AXIS_MAP(0,  1)
 *    #define AXIS_MAP_Y  AXIS_MAP(1,  1)
 *    #define AXIS_MAP_Z  AXIS_MAP(2,  1)
 *
 * 2. 传感器旋转90度: X向前，Y向上(原Z)，Z向后(原-Y反向)
 *    #define AXIS_MAP_X  AXIS_MAP(0,  1)
 *    #define AXIS_MAP_Y  AXIS_MAP(2,  1)
 *    #define AXIS_MAP_Z  AXIS_MAP(1, -1)
 * ===================================================== */
#define AXIS_MAP(axis, sign)  ((axis) | ((sign < 0) ? 0x80 : 0x00))

// 当前轴配置（默认标准配置，如有问题请修改）
#define AXIS_MAP_X  AXIS_MAP(0,  -1)   // 输出X轴 = 传感器X轴正向
#define AXIS_MAP_Y  AXIS_MAP(1,  1)   // 输出Y轴 = 传感器Y轴正向
#define AXIS_MAP_Z  AXIS_MAP(2,  -1)   // 输出Z轴 = 传感器Z轴正向

/**
 * @brief 四元数结构体 q = w + xi + yj + zk
 */
typedef struct {
    float q0;  // w (标量部分)
    float q1;  // x
    float q2;  // y
    float q3;  // z
} Quaternion_t;

/**
 * @brief 欧拉角结构体
 */
typedef struct {
    float roll;   // 横滚角 (-180 ~ 180)
    float pitch;  // 俯仰角 (-90 ~ 90)
    float yaw;    // 偏航角 (-180 ~ 180)
} EulerAngle_t;

/**
 * @brief 四元数姿态解算器结构体
 */
typedef struct {
    Quaternion_t quat;        // 当前四元数姿态
    float integralFBx;        // 误差积分 X
    float integralFBy;        // 误差积分 Y
    float integralFBz;        // 误差积分 Z
    uint8_t inited;           // 初始化标志
} MahonyAHRS_t;

/* API 函数声明 */

/**
 * @brief 初始化 Mahony AHRS
 * @param ahrs Mahony 结构体指针
 */
void Mahony_Init(MahonyAHRS_t *ahrs);

/**
 * @brief 应用轴重映射
 * @param src[3] 输入数据 [x, y, z]
 * @param dst[3] 输出数据 [x, y, z]
 */
void Axis_Remapping(const float src[3], float dst[3]);

/**
 * @brief Mahony 算法更新姿态（6轴）
 * @param ahrs    Mahony 结构体指针
 * @param gx, gy, gz 陀螺仪数据 (rad/s)
 * @param ax, ay, az 加速度计数据 (g)，归一化后
 * @param dt      采样时间间隔 (秒)
 * @note 加速度计数据应在调用前归一化（模长为1）
 */
void Mahony_Update(MahonyAHRS_t *ahrs,
                   float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float dt);

/**
 * @brief Mahony 算法更新姿态（9轴，含磁力计）
 * @param ahrs    Mahony 结构体指针
 * @param gx, gy, gz 陀螺仪数据 (rad/s)
 * @param ax, ay, az 加速度计数据 (g)，归一化后
 * @param mx, my, mz 磁力计数据 (µT)，归一化后
 * @param dt      采样时间间隔 (秒)
 * @note 磁力计数据可显著改善Yaw漂移
 */
void Mahony_Update9Axis(MahonyAHRS_t *ahrs,
                        float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float mx, float my, float mz,
                        float dt);

/**
 * @brief 四元数转欧拉角
 * @param quat 四元数结构体指针
 * @param euler 欧拉角结构体指针
 */
void Quaternion_ToEuler(const Quaternion_t *quat, EulerAngle_t *euler);

/**
 * @brief 获取当前姿态（欧拉角）
 * @param ahrs  Mahony 结构体指针
 * @param euler 欧拉角结构体指针
 */
void Mahony_GetEuler(const MahonyAHRS_t *ahrs, EulerAngle_t *euler);

/**
 * @brief 获取当前四元数
 * @param ahrs Mahony 结构体指针
 * @param quat 四元数结构体指针
 */
void Mahony_GetQuaternion(const MahonyAHRS_t *ahrs, Quaternion_t *quat);

/**
 * @brief 快速平方根倒数（用于归一化）
 * @param x 输入值
 * @return 1 / sqrt(x)
 */
float Fast_InvSqrt(float x);

#endif
