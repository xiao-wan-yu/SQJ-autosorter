#ifndef __GY53_H
#define __GY53_H

//在uart.c文件中，能实现利用GY53测距模块获取前方障碍物距离（连续模式+串口中断接收）

uint16_t GY53_GetDistance_PWM(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

#endif
