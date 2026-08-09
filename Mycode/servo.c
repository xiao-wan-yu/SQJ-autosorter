#include<stm32f4xx_hal.h>
#include "servo.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"

extern TIM_HandleTypeDef htim4;

/**
  * @brief 控制舵机角度
  * @param SERVO_Num SERVOx 舵机号码 x取值为1~4
  * @param SERVO_TargetAngle 舵机目标角度(注意不要超过该舵机的最大角度)
  * @attention 使用舵机前要确保.h文件中的舵机号码对应的最大转动角度是正确匹配的
  */
void SERVO_Control(uint8_t SERVO_Num, float SERVO_TargetAngle){
  switch(SERVO_Num){
    case SERVO1:
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, SERVO_TargetAngle/SERVO1_MaxAngle * 2000 + 500);
      break;
    case SERVO2:
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, SERVO_TargetAngle/SERVO2_MaxAngle * 2000 + 500);
      break;
    case SERVO3:
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, SERVO_TargetAngle/SERVO3_MaxAngle * 2000 + 500);
      break;
    case SERVO4:
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, SERVO_TargetAngle/SERVO4_MaxAngle * 2000 + 500);
      break;
    default:
      break;
  }
}

