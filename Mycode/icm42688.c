#include "icm42688.h"
#include <stdint.h>
#include "./icm42688/icm42688.h"
#include "./icm42688/quaternion.h"
#include "stm32f4xx_hal.h"
#include <math.h>

ICM42688_TYPE ICM42688_Data;


icm42688SensorData_t sensor;
MahonyAHRS_t mahony;
/**
  * @brief 初始化ICM42688，准备接收三个欧拉角数据
  * @retval 初始化正常返回0，初始化失败返回1，陀螺仪校准时不稳定返回2
  * @attention 调用该函数时要确保小车稳定，在初始化成功后再进行更新
  */
uint8_t ICM42688Mahony_Init(void){
  uint8_t state;
  float gyro_sum[3] = {0};
  uint16_t stable_count = 0;

  if (bsp_Icm42688Init() != 0)
  {
      return 1; //初始化失败
  }
  // 陀螺仪校准（改进版：带静态检测）
  HAL_Delay(1000);
  // HAL_Delay(1000);  // 等待用户放稳设备
  // 第一阶段：快速采样，检测设备是否静止
  for (uint16_t i = 0; i < 100; i++) {
      bsp_IcmGetAllSensorData(&sensor);
      gyro_sum[0] += sensor.gx;
      gyro_sum[1] += sensor.gy;
      gyro_sum[2] += sensor.gz;
      HAL_Delay(5);
  }
  // 计算初步平均值
  ICM42688_Data.gyro_offset[0] = gyro_sum[0] / 100.0f;
  ICM42688_Data.gyro_offset[1] = gyro_sum[1] / 100.0f;
  ICM42688_Data.gyro_offset[2] = gyro_sum[2] / 100.0f;
  // 第二阶段：精细校准，只采集静止时的数据
  for (uint16_t i = 0; i < ICM42688_StaticCalibrationTime; i++) { //这里原来是400
      bsp_IcmGetAllSensorData(&sensor);
      // 检测是否静止（与平均值偏差小于 0.5 deg/s）
      if (fabsf(sensor.gx - ICM42688_Data.gyro_offset[0]) < 0.5f &&
          fabsf(sensor.gy - ICM42688_Data.gyro_offset[1]) < 0.5f &&
          fabsf(sensor.gz - ICM42688_Data.gyro_offset[2]) < 0.5f) {
          ICM42688_Data.gyro_offset[0] += sensor.gx;
          ICM42688_Data.gyro_offset[1] += sensor.gy;
          ICM42688_Data.gyro_offset[2] += sensor.gz;
          stable_count++;
      }
      HAL_Delay(5);
  }
  if (stable_count > 0) {
      ICM42688_Data.gyro_offset[0] /= stable_count;
      ICM42688_Data.gyro_offset[1] /= stable_count;
      ICM42688_Data.gyro_offset[2] /= stable_count;
      state = 0; //初始化正常
  } else {
      state = 2; //陀螺仪校准时不稳定
  }
  Mahony_Init(&mahony);
  return state;
}

/**
  * @brief ICM42688+Mahony算法定时更新（根据200Hz的采样频率，建议5ms调用一次）
  */
uint8_t ICM42688Mahony_Update(void){
  // 临时变量用于轴重映射
  float acc_src[3], acc_dst[3];
  float gyro_src[3], gyro_dst[3];
  float dt = 0.005; //代表0.005s
  
  // 获取 IMU 传感器数据
  bsp_IcmGetAllSensorData(&sensor);
  // 应用轴重映射到加速度计
  acc_src[0] = sensor.ax;
  acc_src[1] = sensor.ay;
  acc_src[2] = sensor.az;
  Axis_Remapping(acc_src, acc_dst);
  // 陀螺仪减去零偏后再重映射
  gyro_src[0] = sensor.gx - ICM42688_Data.gyro_offset[0];
  gyro_src[1] = sensor.gy - ICM42688_Data.gyro_offset[1];
  gyro_src[2] = sensor.gz - ICM42688_Data.gyro_offset[2];
  Axis_Remapping(gyro_src, gyro_dst);
  // 加速度计归一化
  float acc_norm = Fast_InvSqrt(acc_dst[0] * acc_dst[0] + acc_dst[1] * acc_dst[1] + acc_dst[2] * acc_dst[2]);
  // 陀螺仪转弧度
  float gx_rad = gyro_dst[0] * DEG_TO_RAD;
  float gy_rad = gyro_dst[1] * DEG_TO_RAD;
  float gz_rad = gyro_dst[2] * DEG_TO_RAD;
  Mahony_Update(&mahony,
                gx_rad, gy_rad, gz_rad,
                acc_dst[0] * acc_norm,
                acc_dst[1] * acc_norm,
                acc_dst[2] * acc_norm,
                dt);
  return 0;
}

/**
  * @brief 接收经过Mahony算法处理得到的三个欧拉角数据
  * @retval 接收正常，返回0
  */
uint8_t ICM42688Mahony_GetAngle(void){
  EulerAngle_t euler;
  Mahony_GetEuler(&mahony, &euler);
  ICM42688_Data.pitch = euler.pitch;
  ICM42688_Data.roll = euler.roll;
  ICM42688_Data.yaw = euler.yaw;
  return 0;
}

/**
  * @brief 初始化ICM42688，准备接收三个加速度、三个角速度的原始数据
  * @retval 初始化成功返回0，初始化失败返回1
  */
uint8_t ICM42688_Init(void){
  int8_t retval;
  retval = bsp_Icm42688Init();
  if(retval == -1) retval = 1;
  return (uint8_t)retval;
}

/**
  * @brief 接收ICM42688的三轴角速度原始数据，并推算出实际三轴角速度
  * @retval 接收正常，返回0
  */
uint8_t ICM42688_GetGyros(void){
  icm42688RawData_t temp;
  bsp_IcmGetGyroscope(&temp, &ICM42688_Data.gyrosx, &ICM42688_Data.gyrosy, &ICM42688_Data.gyrosz);
  return 0;
}

/**
  * @brief 接收ICM4268始8的三轴加速度原数据，并推算出实际三轴加速度和总加速度
  * @retval 接收正常，返回0
  */
uint8_t ICM42688_GetAcc(void){
  icm42688RawData_t temp;
  bsp_IcmGetAccelerometer(&temp, &ICM42688_Data.accx, &ICM42688_Data.accy, &ICM42688_Data.accz);
  ICM42688_Data.acc = sqrt(ICM42688_Data.accx*ICM42688_Data.accx + ICM42688_Data.accy*ICM42688_Data.accy
                       + ICM42688_Data.accz*ICM42688_Data.accz);
  return 0;
}

