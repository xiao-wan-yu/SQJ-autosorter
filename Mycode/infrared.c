/**
  * @brief 红外传感器模块化文件，可用于检测是否有障碍物、区分明显不同的颜色（作用范围：0.1cm~2.5cm）
  *
  * @attention 调节电位器可以调节灵敏度（探测距离）,逆时针旋转电位器减小探测距离
  */

#include <stm32f4xx_hal.h>
#include "main.h"
#include "stm32f4xx_hal_gpio.h"

/**
  * @brief 检测反射式红外传感器是否检测到了障碍物
  * @retval 有障碍物返回1，没有障碍物返回0
  *
  */
uint8_t INFRARED_Barrier(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
    if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)) return 0; //高电平--没有障碍物
    else return 1;  //低电平--有障碍物
}

/**
  * @brief 检测反射式红外传感器是否检测到了白色或浅色
  * @retval 白色或浅色返回1，深色返回0
  *
  */
uint8_t INFRARED_White(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
    if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)) return 0; //高电平--深色
    else return 1;  //低电平--白色或浅色
}
