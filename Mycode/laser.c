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
#include "delay.h"

/**
  * @brief 内部函数：连续采样消抖，返回稳定后的引脚电平
  * @note 快速初检：读到高电平（无目标）立即返回，保持零开销
  *       读到低电平后连续采样，需连续3次(间隔1ms)低电平才确认，
  *       消除激光输出在检测边界附近的信号抖动（偶发高电平会重置计数）
  * @retval 稳定高电平返回1，稳定低电平返回0
  */
static uint8_t LASER_StableLevel(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
    /*快速初检：无目标时立即返回，保持零开销*/
    if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) != GPIO_PIN_RESET){
        return 1;  //高电平--没有障碍物/深色
    }

    /*初检为低电平：连续采样，需连续3次(间隔1ms)低电平才确认
      激光在检测边界附近输出会抖动，偶发高电平会重置计数*/
    uint8_t stable = 0;
    for(uint8_t i = 0; i < 6; i++){  //最多采样6次（5ms窗口）
        delay_us(1000);
        if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET){
            if(++stable >= 3){  //连续3次低电平 → 确认
                return 0;
            }
        }else{
            stable = 0;  //读到高电平（抖动），重置连续计数
        }
    }
    return 1;  //5ms内未能确认，视为无目标（高电平）
}

/**
  * @brief 检测激光漫反射传感器是否检测到了障碍物（带消抖）
  * @retval 有障碍物返回1，没有障碍物返回0
  *
  */
uint8_t LASER_Barrier(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
    return !LASER_StableLevel(GPIOx, GPIO_Pin); //低电平--有障碍物
}

/**
  * @brief 检测激光漫反射传感器是否检测到了白色或浅色（带消抖）
  * @retval 白色或浅色返回1，深色返回0
  *
  */
uint8_t LASER_White(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
    return !LASER_StableLevel(GPIOx, GPIO_Pin); //低电平--白色或浅色
}
