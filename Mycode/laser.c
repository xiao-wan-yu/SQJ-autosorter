/**
  * @brief 激光漫反射传感器模块化文件，可用于检测是否有障碍物、区分明显不同的颜色（作用范围：第五代：0.5cm~200cm；第四代：0.5cm~150cm）
  *
  * @attention 激光漫反射传感器使用前须先固定角度（20~80度），然后调试电位器
  *             测试时用的是第四代激光漫反射传感器，识别颜色效果尚可，但是识别障碍物较难调试，
  *             建议采用第五代激光漫反射传感器看是否方便调试
  */
#include <stm32f4xx_hal.h>
#include "main.h"
#include "stm32f4xx_hal_gpio.h"

/**
  * @brief 检测激光漫反射传感器是否检测到了障碍物
  * @retval 有障碍物返回1，没有障碍物返回0
  *
  */
uint8_t LASER_Barrier(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
    if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)) return 0; //高电平--没有障碍物
    else return 1;  //低电平--有障碍物
}

/**
  * @brief 检测激光漫反射传感器是否检测到了白色或浅色
  * @retval 白色或浅色返回1，深色返回0
  *
  */
uint8_t LASER_White(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
    if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)) return 0; //高电平--深色
    else return 1;  //低电平--白色或浅色
}
