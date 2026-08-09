/**
  * @brief 超声波传感器模块化文件，可用于检测前方障碍物距离（探测距离：2cm-450cm）
  */

#include <stm32f4xx_hal.h>
#include "ultrasonic.h"
#include "delay.h"
#include "stm32_hal_legacy.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"

//定时器6 每1us CNT计数值加1
extern TIM_HandleTypeDef htim6;

/**
  * @brief 利用超声波传感器获取前方障碍物距离（阻塞式）
  * @retval 前方障碍物距离（单位：cm）（精度：0.3cm）
  */
float ULTRASONIC_GetDistance(void){
  uint32_t highlevletime; //高电平期间CNT计数值
  HAL_GPIO_WritePin(ULTRASONIC_Trig_GPIO_Port, ULTRASONIC_Trig_Pin, GPIO_PIN_SET);
  delay_us(20);
  HAL_GPIO_WritePin(ULTRASONIC_Trig_GPIO_Port, ULTRASONIC_Trig_Pin, GPIO_PIN_RESET);

  while(HAL_GPIO_ReadPin(ULTRASONIC_Echo_GPIO_Port, ULTRASONIC_Echo_Pin) == 0); //等待高电平
  __HAL_TIM_SetCounter(&htim6, 0);
  HAL_TIM_Base_Start(&htim6); //开启定时器6时基单元，开始计时
  while(HAL_GPIO_ReadPin(ULTRASONIC_Echo_GPIO_Port, ULTRASONIC_Echo_Pin) == 1); //等待高电平结束
  HAL_TIM_Base_Stop(&htim6);
  highlevletime = __HAL_TIM_GetCounter(&htim6);
  return (highlevletime * 0.000001 * 340 /2) * 100; //距离=（高电平时间*声速）/ 2 (单位：cm)
}

