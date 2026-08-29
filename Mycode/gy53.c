/**
  * @brief GY53测距模块模块化文件，可用于检测前方障碍物距离（探测距离：0-2000mm）
  */
  
#include <stm32f4xx_hal.h>
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_uart.h"

//定时器6 每1us CNT计数值加1
extern TIM_HandleTypeDef htim6;

//在uart.c文件中，能实现利用GY53测距模块获取前方障碍物距离（连续模式+串口中断接收）

/**
  * @brief 利用GY53测距模块获取前方障碍物距离（连续模式+PWM模式接收）（阻塞式）
  * @retval 前方障碍物距离（单位：mm）
  */
uint16_t GY53_GetDistance_PWM(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
  uint32_t t0;
  /* 等待高电平：超过50ms仍无目标 → 返回超量程2000mm（防阻塞） */
  t0 = HAL_GetTick();
  while(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == 0){
    if((HAL_GetTick() - t0) > 50) return 2000;
  }
  __HAL_TIM_SetCounter(&htim6, 0);
  HAL_TIM_Base_Start(&htim6); //开启定时器6时基单元，开始计时
  /* 等待高电平结束：超过25ms(≈2500mm) 视为超量程/无目标（防阻塞） */
  t0 = HAL_GetTick();
  while(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == 1){
    if((HAL_GetTick() - t0) > 25) return 2000;
  }
  HAL_TIM_Base_Stop(&htim6);
  return  (__HAL_TIM_GetCounter(&htim6) * 1 / 10);  //距离(mm)=高电平时间(ms)*100=高电平时间(us)/10 （单位：mm）
} 
