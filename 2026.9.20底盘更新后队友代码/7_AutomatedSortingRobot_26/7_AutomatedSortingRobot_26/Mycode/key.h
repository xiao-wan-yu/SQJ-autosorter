#ifndef __KEY_H
#define __KEY_H

#define KEY_NO -1

/*没有按下按键返回0，按下按键返回1（直到松手才退出函数）*/
uint8_t KEY_ONE(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
int8_t KEY_ALL(void);

#endif
