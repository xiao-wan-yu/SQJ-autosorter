#ifndef __ICM42688_H
#define __ICM42688_H

#include "stm32f4xx_hal.h"

//静态校准时间：ICM42688_StaticCalibrationTime * 5ms；校准时间越长，yaw越稳定，建议值：400
#define ICM42688_StaticCalibrationTime  2000

typedef struct{
  float pitch;    //俯仰角    范围:-90.0° <---> +90.0°
  float roll;     //横滚角    范围:-180.0°<---> +180.0°
  float yaw;      //偏航角    范围:-180.0°<---> +180.0°
  float acc;      //总加速度
  float accx;     //x轴加速度
  float accy;     //y轴加速度
  float accz;     //z轴加速度
  float gyrosx;   //x轴角速度
  float gyrosy;   //y轴角速度
  float gyrosz;   //z轴角速度
  float gyro_offset[3]; //陀螺仪零漂（icm42688配合mahony算法初始化时测得）
}ICM42688_TYPE;

extern ICM42688_TYPE ICM42688_Data;

uint8_t ICM42688Mahony_Init(void);
uint8_t ICM42688Mahony_Update(void);
uint8_t ICM42688Mahony_GetAngle(void);
uint8_t ICM42688_Init(void);
uint8_t ICM42688_GetGyros(void);
uint8_t ICM42688_GetAcc(void);

#endif
