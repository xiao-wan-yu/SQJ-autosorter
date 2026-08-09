#include "quaternion.h"
#include <math.h>

/**
 * @brief 快速平方根倒数（Quake III 算法优化版）
 * @param x 输入值（必须 > 0）
 * @return 1 / sqrt(x)
 */
float Fast_InvSqrt(float x)
{
    float halfx = 0.5f * x;
    int i = *(int*)&x;            // 浮点数位表示
    i = 0x5f3759df - (i >> 1);    // 魔术常数
    x = *(float*)&i;              // 转回浮点数
    x = x * (1.5f - halfx * x * x); // 牛顿迭代一次
    return x;
}

/**
 * @brief 应用轴重映射
 * @param src[3] 输入数据 [x, y, z]
 * @param dst[3] 输出数据 [x, y, z]
 *
 * 根据 AXIS_MAP_X/Y/Z 宏定义进行轴重映射
 */
void Axis_Remapping(const float src[3], float dst[3])
{
    // 解析映射配置：bit[7]=符号(1=负), bit[1:0]=源轴(0/1/2)
    int map_x = AXIS_MAP_X;
    int map_y = AXIS_MAP_Y;
    int map_z = AXIS_MAP_Z;

    // X轴映射
    dst[0] = src[map_x & 0x03];
    if (map_x & 0x80) dst[0] = -dst[0];

    // Y轴映射
    dst[1] = src[map_y & 0x03];
    if (map_y & 0x80) dst[1] = -dst[1];

    // Z轴映射
    dst[2] = src[map_z & 0x03];
    if (map_z & 0x80) dst[2] = -dst[2];
}

/**
 * @brief 初始化 Mahony AHRS
 * @param ahrs Mahony 结构体指针
 */
void Mahony_Init(MahonyAHRS_t *ahrs)
{
    // 初始化为单位四元数（无旋转状态）
    ahrs->quat.q0 = 1.0f;
    ahrs->quat.q1 = 0.0f;
    ahrs->quat.q2 = 0.0f;
    ahrs->quat.q3 = 0.0f;

    // 清零积分误差
    ahrs->integralFBx = 0.0f;
    ahrs->integralFBy = 0.0f;
    ahrs->integralFBz = 0.0f;

    ahrs->inited = 1;
}

/**
 * @brief Mahony 算法更新姿态（6轴：加速度计 + 陀螺仪）
 * @param ahrs    Mahony 结构体指针
 * @param gx, gy, gz 陀螺仪数据 (rad/s)
 * @param ax, ay, az 加速度计数据，已归一化
 * @param dt      采样时间间隔 (秒)
 *
 * 算法原理：
 * 1. 用加速度计测量重力方向，与四元数推算的重力方向比较得到误差
 * 2. 用 PI 控制器修正陀螺仪漂移
 * 3. 四元数微分方程更新姿态
 */
void Mahony_Update(MahonyAHRS_t *ahrs,
                   float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float dt)
{
    float q0 = ahrs->quat.q0;
    float q1 = ahrs->quat.q1;
    float q2 = ahrs->quat.q2;
    float q3 = ahrs->quat.q3;

    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    // float px, py, pz;
    float q0_last, q1_last, q2_last, q3_last;

    /* ========== 第一步：计算加速度计归一化 ========== */
    // 注：调用时应确保 ax, ay, az 已归一化，这里再次归一化确保安全
    norm = Fast_InvSqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    /* ========== 第二步：用四元数推算重力方向（机体坐标系） ========== */
    // 旋转矩阵 R，将世界坐标系的重力 [0, 0, 1] 转换到机体坐标系
    // [vx]   [1 - 2(q2² + q3²)    2(q1q2 - q0q3)    2(q1q3 + q0q2)  ] [0]
    // [vy] = [2(q1q2 + q0q3)    1 - 2(q1² + q3²)    2(q2q3 - q0q1)  ] [0]
    // [vz]   [2(q1q3 - q0q2)    2(q2q3 + q0q1)    1 - 2(q1² + q2²)] [1]
    // 简化后：
    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    /* ========== 第三步：计算方向误差（加速度计测量 vs 推算值） ========== */
    // 叉乘得到误差向量（垂直于两者，表示需要旋转的轴）
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    /* ========== 第四步：PI 控制器修正陀螺仪漂移 ========== */
    // 累加积分项
    ahrs->integralFBx += MAHONY_KI * ex * dt;
    ahrs->integralFBy += MAHONY_KI * ey * dt;
    ahrs->integralFBz += MAHONY_KI * ez * dt;

    // P 控制器 + 积分项，得到修正后的角速度
    gx = gx + MAHONY_KP * ex + ahrs->integralFBx;
    gy = gy + MAHONY_KP * ey + ahrs->integralFBy;
    gz = gz + MAHONY_KP * ez + ahrs->integralFBz;

    /* ========== 第五步：四元数微分方程更新 ========== */
    // q_dot = 0.5 * q * ω  (四元数乘以角速度四元数)
    // [q0_dot]   [0  -gx -gy -gz] [q0]
    // [q1_dot] = [gx  0   gz -gy] [q1] * 0.5
    // [q2_dot]   [gy -gz  0   gx] [q2]
    // [q3_dot]   [gz  gy -gx  0 ] [q3]

    q0_last = q0 - 0.5f * (q1 * gx + q2 * gy + q3 * gz) * dt;
    q1_last = q1 + 0.5f * (q0 * gx + q2 * gz - q3 * gy) * dt;
    q2_last = q2 + 0.5f * (q0 * gy - q1 * gz + q3 * gx) * dt;
    q3_last = q3 + 0.5f * (q0 * gz + q1 * gy - q2 * gx) * dt;

    /* ========== 第六步：四元数归一化 ========== */
    norm = Fast_InvSqrt(q0_last * q0_last + q1_last * q1_last +
                        q2_last * q2_last + q3_last * q3_last);
    ahrs->quat.q0 = q0_last * norm;
    ahrs->quat.q1 = q1_last * norm;
    ahrs->quat.q2 = q2_last * norm;
    ahrs->quat.q3 = q3_last * norm;
}

/**
 * @brief Mahony 算法更新姿态（9轴：加速度计 + 陀螺仪 + 磁力计）
 * @param ahrs    Mahony 结构体指针
 * @param gx, gy, gz 陀螺仪数据 (rad/s)
 * @param ax, ay, az 加速度计数据，已归一化
 * @param mx, my, mz 磁力计数据 (µT)，已归一化
 * @param dt      采样时间间隔 (秒)
 *
 * 算法原理：
 * 1. 用加速度计测量重力方向，与四元数推算的重力方向比较得到误差
 * 2. 用磁力计测量地磁场方向，与四元数推算的地磁场方向比较得到误差
 * 3. 用 PI 控制器修正陀螺仪漂移（包括Yaw轴）
 * 4. 四元数微分方程更新姿态
 */
void Mahony_Update9Axis(MahonyAHRS_t *ahrs,
                        float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float mx, float my, float mz,
                        float dt)
{
    float q0 = ahrs->quat.q0;
    float q1 = ahrs->quat.q1;
    float q2 = ahrs->quat.q2;
    float q3 = ahrs->quat.q3;

    float norm;
    float vx, vy, vz;
    float wx, wy, wz;
    float ex, ey, ez;
    // float px, py, pz;
    float q0_last, q1_last, q2_last, q3_last;

    /* ========== 第一步：归一化加速度计和磁力计数据 ========== */
    // 加速度计归一化
    norm = Fast_InvSqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    // 磁力计归一化
    norm = Fast_InvSqrt(mx * mx + my * my + mz * mz);
    mx *= norm;
    my *= norm;
    mz *= norm;

    /* ========== 第二步：用四元数推算重力和地磁场方向（机体坐标系） ========== */
    // 重力方向（加速度计参考）
    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // 地磁场方向（磁力计参考，假设初始磁场指向北方）
    // 旋转矩阵 R，将世界坐标系的地磁场 [1, 0, 0] 转换到机体坐标系
    wx = 1.0f - 2.0f * (q2 * q2 + q3 * q3);  // Hx
    wy = 2.0f * (q1 * q2 + q0 * q3);        // Hy
    wz = 2.0f * (q1 * q3 - q0 * q2);        // Hz

    /* ========== 第三步：计算方向误差 ========== */
    // 加速度计误差（Roll/Pitch修正）
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    // 磁力计误差（Yaw修正）
    // 将磁场测量值投影到水平面
    float hx = wx;
    float hy = wy;
    float hz = wz;

    // 计算磁力计误差（与推算的地磁场方向比较）
    ex += hy * wz - hz * wy;
    ey += hz * wx - hx * wz;
    ez += hx * wy - hy * wx;

    /* ========== 第四步：PI 控制器修正陀螺仪漂移 ========== */
    // 累加积分项
    ahrs->integralFBx += MAHONY_KI * ex * dt;
    ahrs->integralFBy += MAHONY_KI * ey * dt;
    ahrs->integralFBz += MAHONY_KI * ez * dt;

    // P 控制器 + 积分项，得到修正后的角速度
    gx = gx + MAHONY_KP * ex + ahrs->integralFBx;
    gy = gy + MAHONY_KP * ey + ahrs->integralFBy;
    gz = gz + MAHONY_KP * ez + ahrs->integralFBz;

    /* ========== 第五步：四元数微分方程更新 ========== */
    q0_last = q0 - 0.5f * (q1 * gx + q2 * gy + q3 * gz) * dt;
    q1_last = q1 + 0.5f * (q0 * gx + q2 * gz - q3 * gy) * dt;
    q2_last = q2 + 0.5f * (q0 * gy - q1 * gz + q3 * gx) * dt;
    q3_last = q3 + 0.5f * (q0 * gz + q1 * gy - q2 * gx) * dt;

    /* ========== 第六步：四元数归一化 ========== */
    norm = Fast_InvSqrt(q0_last * q0_last + q1_last * q1_last +
                        q2_last * q2_last + q3_last * q3_last);
    ahrs->quat.q0 = q0_last * norm;
    ahrs->quat.q1 = q1_last * norm;
    ahrs->quat.q2 = q2_last * norm;
    ahrs->quat.q3 = q3_last * norm;
}

/**
 * @brief 四元数转欧拉角
 * @param quat 四元数结构体指针
 * @param euler 欧拉角结构体指针
 *
 * 转换公式（ZYX 顺序，即 Yaw -> Pitch -> Roll）：
 * roll  = atan2(2(q0q1 + q2q3), 1 - 2(q1² + q2²))
 * pitch = asin(2(q0q2 - q3q1))
 * yaw   = atan2(2(q0q3 + q1q2), 1 - 2(q2² + q3²))
 */
void Quaternion_ToEuler(const Quaternion_t *quat, EulerAngle_t *euler)
{
    float q0 = quat->q0;
    float q1 = quat->q1;
    float q2 = quat->q2;
    float q3 = quat->q3;

    // Roll (φ): 绕 X 轴旋转
    euler->roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                         1.0f - 2.0f * (q1 * q1 + q2 * q2));

    // Pitch (θ): 绕 Y 轴旋转
    // asinf 参数限制在 [-1, 1]，避免 NaN
    float pitch_arg = 2.0f * (q0 * q2 - q3 * q1);
    if (pitch_arg > 1.0f) pitch_arg = 1.0f;
    if (pitch_arg < -1.0f) pitch_arg = -1.0f;
    euler->pitch = asinf(pitch_arg);

    // Yaw (ψ): 绕 Z 轴旋转
    euler->yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                        1.0f - 2.0f * (q2 * q2 + q3 * q3));

    // 转换为角度
    euler->roll *= RAD_TO_DEG;
    euler->pitch *= RAD_TO_DEG;
    euler->yaw *= RAD_TO_DEG;
}

/**
 * @brief 获取当前姿态（欧拉角）
 * @param ahrs  Mahony 结构体指针
 * @param euler 欧拉角结构体指针
 */
void Mahony_GetEuler(const MahonyAHRS_t *ahrs, EulerAngle_t *euler)
{
    Quaternion_ToEuler(&ahrs->quat, euler);
}

/**
 * @brief 获取当前四元数
 * @param ahrs Mahony 结构体指针
 * @param quat 四元数结构体指针
 */
void Mahony_GetQuaternion(const MahonyAHRS_t *ahrs, Quaternion_t *quat)
{
    quat->q0 = ahrs->quat.q0;
    quat->q1 = ahrs->quat.q1;
    quat->q2 = ahrs->quat.q2;
    quat->q3 = ahrs->quat.q3;
}
