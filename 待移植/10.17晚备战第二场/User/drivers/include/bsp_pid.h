#ifndef __BSP_PID_H
#define	__BSP_PID_H
#include "stm32f10x.h"
#include "usart.h"   
#include <stdio.h>
#include <stdlib.h>

/*pid*/
typedef struct
{
    float target_val;           //目标值
	  float target_dif_val;       //目标值微调量
    float actual_val;        		//实际值
    float err;             			//定义偏差值
    float err_last;          		//定义上一个偏差值
    float Kp,Ki,Kd;          		//定义比例、积分、微分系数
    float integral;          		//定义积分值
	  float integral_max;         //积分饱和
	  float int_separation;       //积分分离条件
	  float dead_space;           //闭环死区条件
}pid_t;

extern volatile uint8_t x_set_speed_flag ;
extern volatile uint8_t y_set_speed_flag ;

extern void PID_param_init();
extern void set_pid_target(pid_t *pid,float temp_val);
extern float get_pid_target(pid_t *pid);
extern void set_p_i_d(pid_t *pid,float p, float i, float d);
extern float PID_realize(pid_t *pid,float temp_val);
extern void time_period_fun(void);

#endif
