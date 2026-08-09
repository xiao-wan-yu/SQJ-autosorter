/**
  * @brief 为了使用方便，将MPU6050读取原始数据的mpu文件和读取欧拉角的系列dmp文件里面需要用到的函数进行再度封装
  */
#include "./mpu6050/inv_mpu.h"
#include "./mpu6050/mpu.h"
#include "mpu6050.h"
#include <math.h>

MPU6050_TYPE MPU6050_Data;

//陀螺仪灵敏度表 计算公式：gyro_sensitivity[MPU6050_Gyro_Fsr] = 65536.0 / (2 * 250.0 * pow(2.0,MPU6050_Gyro_Fsr))
static const float gyro_sensitivity[] = {
    131.0f,  // ±250°/s
    65.5f,   // ±500°/s
    32.8f,   // ±1000°/s
    16.4f    // ±2000°/s
};
//加速度灵敏度表 计算公式：计算原理同上
static const float acc_sensitivity[] = {
    16384.0f,  // ±2g
    8192.0f,   // ±4g
    4096.0f,   // ±8g
    2048.0f    // ±16g
};

/**
  * @brief 初始化MPU6050，准备接收三个欧拉角数据
  * @retval 初始化正常，返回0；初始化失败，返回其他数字
  */
uint8_t MPU6050DMP_Init(void){
    return mpu_dmp_init();
}

/**
  * @brief 接收经过DMP库处理得到的三个欧拉角数据
  * @retval 接收正常，返回0；接收失败，返回其他数字
  */
uint8_t MPU6050DMP_GetAngle(void){
    return mpu_dmp_get_data(&MPU6050_Data.pitch, &MPU6050_Data.roll, &MPU6050_Data.yaw);
}


/**
  * @brief 初始化MPU6050，准备接收三个加速度、三个角速度的原始数据
  * @param gyro_fsr 陀螺仪满量程范围  0：±250dps;1：±500dps;2：±1000dps;3：±2000dps
  * @param accel_fsr 加速度传感器满量程范围  0：±2g;1：±4g;2：±8g;3：±16g
  * @retval 初始化正常，返回0；初始化失败，返回其他数字
  */
uint8_t MPU6050_Init(void){
    return MPU_Init(MPU6050_Gyro_Fsr, MPU6050_Accel_Fsr);
}

/**
  * @brief 接收MPU6050的三轴角速度原始数据，并根据陀螺仪灵敏度表推算出实际三轴角速度
  * @retval 接收正常，返回0；接收失败，返回其他数字
  */
uint8_t MPU6050_GetGyros(void){
    short gx, gy, gz;
    uint8_t ret;

    ret = MPU_Get_Gyroscope(&gx, &gy, &gz);
    if(ret != 0) return ret;

    MPU6050_Data.gyrosx = gx / gyro_sensitivity[MPU6050_Gyro_Fsr];
    MPU6050_Data.gyrosy = gy / gyro_sensitivity[MPU6050_Gyro_Fsr];
    MPU6050_Data.gyrosz = gz / gyro_sensitivity[MPU6050_Gyro_Fsr];

    return 0;
}

/**
  * @brief 接收MPU6050的三轴加速度原始数据，并根据加速度传感器灵敏度表推算出实际三轴加速度和总加速度
  * @retval 接收正常，返回0；接收失败，返回其他数字
  */
uint8_t MPU6050_GetAcc(void){
    short ax, ay, az;
    uint8_t ret;

    ret = MPU_Get_Accelerometer(&ax, &ay, &az);
    if(ret != 0) return ret;

    MPU6050_Data.accx = ax / acc_sensitivity[MPU6050_Accel_Fsr];
    MPU6050_Data.accy = ay / acc_sensitivity[MPU6050_Accel_Fsr];
    MPU6050_Data.accz = az / acc_sensitivity[MPU6050_Accel_Fsr];
    MPU6050_Data.acc = sqrt(MPU6050_Data.accx * MPU6050_Data.accx
         + MPU6050_Data.accy * MPU6050_Data.accy + MPU6050_Data.accz * MPU6050_Data.accz);

    return 0;
}
