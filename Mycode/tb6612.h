#ifndef __TB6612_H
#define __TB6612_H

#include "stm32f4xx_hal.h"

//直流减速电机号码
#define MOTOR_Left    1
#define MOTOR_Right   2

void TB6612_Control(uint8_t MOTOR_Num, int8_t Motor_Speed);

#endif
