#include "nmos.h"
#include "stm32f4xx_hal_gpio.h"

/**
  * @brief 开启指定的nmos管，使其导通，从而给大功率器件供电
  */
void NMOS_ON(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

/**
  * @brief 关闭指定的nmos管，使其截止，从而给大功率器件断电
  */
void NMOS_OFF(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}
