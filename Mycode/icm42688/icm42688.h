#ifndef __BSP_ICM42688_H__
#define __BSP_ICM42688_H__

#include <stdint.h>

#define ICM_USE_I2C

#define ICM42688_DEVICE_CONFIG             0x11
#define ICM42688_TEMP_DATA1                0x1D
#define ICM42688_TEMP_DATA2                0x1E
#define ICM42688_ACCEL_DATA_X1             0x1F
#define ICM42688_GYRO_DATA_X1              0x25
#define ICM42688_INT_STATUS                0x2D  // 中断状态寄存器
#define ICM42688_DATA_RDY_STATUS           0x35  // 数据就绪状态寄存器
#define ICM42688_SIGNAL_PATH_RESET         0x4B
#define ICM42688_PWR_MGMT0                 0x4E
#define ICM42688_GYRO_CONFIG0              0x4F
#define ICM42688_ACCEL_CONFIG0             0x50
#define ICM42688_WHO_AM_I                  0x75
#define ICM42688_REG_BANK_SEL              0x76
#define ICM42688_SENSOR_CONFIG0            0x03
#define ICM42688_GYRO_CONFIG_STATIC3       0x0C
#define ICM42688_GYRO_ACCEL_CONFIG0        0x14
#define ICM42688_ACCEL_CONFIG_STATIC3      0x04

#define ICM42688_ADDRESS                   0xD2

#define AFS_2G  0x03
#define AFS_4G  0x02
#define AFS_8G  0x01
#define AFS_16G 0x00

#define GFS_2000DPS   0x00
#define GFS_1000DPS   0x01
#define GFS_500DPS    0x02
#define GFS_250DPS    0x03
#define GFS_125DPS    0x04
#define GFS_62_5DPS   0x05
#define GFS_31_25DPS  0x06
#define GFS_15_125DPS 0x07

#define AODR_200Hz    0x07
#define AODR_100Hz    0x08

#define GODR_200Hz    0x07
#define GODR_100Hz    0x08

#define AAVG_4X       0x02
#define AAF_1_16_ODR  0x04
#define GAVG_4X       0x02
#define GF_1_16_ODR   0x04

#define ICM42688_ID	 0x47

typedef struct {
    int16_t x, y, z;
} icm42688RawData_t;

typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
} icm42688SensorData_t;

int8_t bsp_Icm42688Init(void);
int8_t bsp_IcmGetTemperature(int16_t* pTemp);
int8_t bsp_IcmGetAccelerometer(icm42688RawData_t* accData, float* ax, float* ay, float* az);
int8_t bsp_IcmGetGyroscope(icm42688RawData_t* gyroData, float* gx, float* gy, float* gz);
int8_t bsp_IcmGetAllSensorData(icm42688SensorData_t* data);

#endif
