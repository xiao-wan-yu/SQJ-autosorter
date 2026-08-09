#ifndef __MPU6050_H
#define __MPU6050_H

typedef struct{
    float pitch;    //俯仰角 精度:0.1°   范围:-90.0° <---> +90.0°
    float roll;     //横滚角 精度:0.1°   范围:-180.0°<---> +180.0°
    float yaw;      //偏航角 精度:0.1°   范围:-180.0°<---> +180.0°
    float acc;      //总加速度
    float accx;     //x轴加速度
    float accy;     //y轴加速度
    float accz;     //z轴加速度
    float gyrosx;   //x轴角速度
    float gyrosy;   //y轴角速度
    float gyrosz;   //z轴角速度
}MPU6050_TYPE;  

extern MPU6050_TYPE MPU6050_Data;   //存放mpu6050的数据

#define MPU6050_Gyro_Fsr 3      //使用MPU6050不带DMP解算时的陀螺仪满量程范围选择  0：±250dps;1：±500dps;2：±1000dps;3：±2000dps
#define MPU6050_Accel_Fsr 0     //使用MPU6050不带DMP解算时的加速度满量程范围选择  0：±2g;1：±4g;2：±8g;3：±16g

uint8_t MPU6050DMP_Init(void);
uint8_t MPU6050DMP_GetAngle(void);
uint8_t MPU6050_Init(void);
uint8_t MPU6050_GetGyros(void);
uint8_t MPU6050_GetAcc(void);

#endif
