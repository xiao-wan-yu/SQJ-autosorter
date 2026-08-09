#include "tb6612.h"
#include "main.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "tb6612.h"

extern TIM_HandleTypeDef htim1;

/**
  * @brief 控制直流减速电机的转向和转速
  * @param MOTOR_Num 电机号码 可选择：MOTOR_Left或MOTOR_Right
  * @param Motor_Speed 电机速度：-100~100
  */
void TB6612_Control(uint8_t MOTOR_Num, int8_t Motor_Speed){
  switch(MOTOR_Num){
    case MOTOR_Left:  //左电机
      if(Motor_Speed >= 0){
        HAL_GPIO_WritePin(TB6612_AIN1_GPIO_Port, TB6612_AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TB6612_AIN2_GPIO_Port, TB6612_AIN2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Motor_Speed);
      }else{
        HAL_GPIO_WritePin(TB6612_AIN1_GPIO_Port, TB6612_AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(TB6612_AIN2_GPIO_Port, TB6612_AIN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -Motor_Speed);
      }
      break;
    case MOTOR_Right: //右电机
      if(Motor_Speed >= 0){
        HAL_GPIO_WritePin(TB6612_BIN1_GPIO_Port, TB6612_BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TB6612_BIN2_GPIO_Port, TB6612_BIN2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, Motor_Speed);
      }else{
        HAL_GPIO_WritePin(TB6612_BIN1_GPIO_Port, TB6612_BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(TB6612_BIN2_GPIO_Port, TB6612_BIN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, -Motor_Speed);
      }
      break;
    default:
      break;
  }
}

