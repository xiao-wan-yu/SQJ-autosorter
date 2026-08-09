#ifndef __LED_H
#define __LED_H

#include "main.h"

void LED_ON(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void LED_OFF(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

#endif
