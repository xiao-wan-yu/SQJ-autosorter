#ifndef __INFRARED_H
#define __INFRARED_H

uint8_t INFRARED_Barrier(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint8_t INFRARED_White(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

#endif
