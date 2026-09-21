#include "led.h"
#include "stm32f4xx_hal.h"
#include "main.h"

void LED_ON(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin,  GPIO_PIN_RESET);
}

void LED_OFF(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin,  GPIO_PIN_SET);
}
