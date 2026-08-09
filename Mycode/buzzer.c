#include "stm32f4xx_hal.h"
#include "main.h"
#include "stm32f4xx_hal_gpio.h"

void BUZZER_On(void){
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
}

void BUZZER_OFF(void){
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
}

