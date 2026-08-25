#include "tb6612.h"
#include "main.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "tb6612.h"

extern TIM_HandleTypeDef htim1;

/**
  * @brief 控制直流减速电机的转向和转速
  * @param MOTOR_Num 电机号码 可选择：MOTOR_Right_Front MOTOR_Right_Back MOTOR_Left_Back MOTOR_Left_Front
  * @param Motor_Speed 电机速度：-1000~1000
  */
void TB6612_Control(uint8_t MOTOR_Num, int16_t Motor_Speed){
  switch(MOTOR_Num){
    case MOTOR_Left_Front:  //左前电机
      if(Motor_Speed >= 0){
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Motor_Speed);
      }else{
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -Motor_Speed);
      }
      break;
    case MOTOR_Left_Back: //左后电机
      if(Motor_Speed >= 0){
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Motor_Speed);
      }else{
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -Motor_Speed);
      }
      break;
    case MOTOR_Right_Back: //右后电机
      if(Motor_Speed >= 0){
        HAL_GPIO_WritePin(CIN1_GPIO_Port, CIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Motor_Speed);
      }else{
        HAL_GPIO_WritePin(CIN1_GPIO_Port, CIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, -Motor_Speed);
      }
      break;
    case MOTOR_Right_Front: //右前电机
      if(Motor_Speed >= 0){
        HAL_GPIO_WritePin(DIN1_GPIO_Port, DIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIN2_GPIO_Port, DIN2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, Motor_Speed);
      }else{
        HAL_GPIO_WritePin(DIN1_GPIO_Port, DIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DIN2_GPIO_Port, DIN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, -Motor_Speed);
      }
      break;
    default:
      break;
  }
}

