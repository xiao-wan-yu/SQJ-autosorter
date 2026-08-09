#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "main.h"

//使用超声波模块之前，先把宏定义切换到对应的引脚,Echo引脚配置为下拉输入模式,Trig引脚配置为推挽输出模式
#define ULTRASONIC_Echo_Pin           SENSOR1_Pin
#define ULTRASONIC_Echo_GPIO_Port     SENSOR1_GPIO_Port
#define ULTRASONIC_Trig_Pin           SENSOR3_Pin
#define ULTRASONIC_Trig_GPIO_Port     SENSOR3_GPIO_Port

float ULTRASONIC_GetDistance(void);

#endif
