#ifndef __TASK_TIMER_H_
#define __TASK_TIMER_H_
/*-------------------------------ÎÄ¼þ°üº¬--------------------------------*/
#include "stm32f10x.h"

void Task_TIM_Init(void);
void Task_TIM_NewState(FunctionalState newstate);
void  RunTaskInTIM_main(void);
#endif





























