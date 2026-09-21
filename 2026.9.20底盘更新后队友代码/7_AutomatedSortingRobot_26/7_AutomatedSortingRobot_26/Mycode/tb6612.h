#ifndef __TB6612_H
#define __TB6612_H

#include "stm32f4xx_hal.h"

//直流减速电机号码
#define MOTOR_Left_Front    1
#define MOTOR_Left_Back     2
#define MOTOR_Right_Back    3
#define MOTOR_Right_Front   4

void TB6612_Control(uint8_t MOTOR_Num, int16_t Motor_Speed);

#endif
