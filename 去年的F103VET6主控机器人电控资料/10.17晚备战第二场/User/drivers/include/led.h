#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"

void LED_GPIO_Config(void);

#define LED_R_ON	GPIO_SetBits(GPIOE, GPIO_Pin_2)
#define LED_R_OFF	GPIO_ResetBits(GPIOE,GPIO_Pin_2)

#define LED_G_ON	GPIO_SetBits(GPIOE, GPIO_Pin_5)
#define LED_G_OFF	GPIO_ResetBits(GPIOE,GPIO_Pin_5)


#endif
