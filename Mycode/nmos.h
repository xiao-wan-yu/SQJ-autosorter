#ifndef __NMOS_H
#define __NMOS_H

#include "main.h"

void NMOS_ON(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void NMOS_OFF(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

#endif
