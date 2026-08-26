#ifndef _PID_TIMER_H_
#define _PID_TIMER_H_
#include "stm32f10x.h" //ºƒ¥Ê∆˜”≥…‰”Î∫Í∂®“Â


#define set_pidtim_period(t)         TIM_Cmd(TIM6, DISABLE);TIM_SetAutoreload(TIM6, (t)*1000-1);TIM_Cmd(TIM6, ENABLE)
void PID_BasicTim_Init(void);

#endif
