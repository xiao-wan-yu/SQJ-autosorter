#include "bsp_pid.h"
#include "math.h"
//#include "bsp_key.h" 
#include "protocol.h"


/**
  * @brief  PID参数初始化
	*	@note 	无
  * @retval 无
  */
extern	volatile float angle;                                                      //陀螺仪实时角度
void PID_param_init()
{

}

/**
  * @brief  设置目标值
  * @param  val		目标值
	*	@note 	无
  * @retval 无
  */
void set_pid_target(pid_t* pid,float temp_val)
{
  pid->target_val = temp_val;    // 设置当前的目标值
	//pid->integral = 0;
}

/**
  * @brief  获取目标值
  * @param  无
	*	@note 	无
  * @retval 目标值
  */
float get_pid_target(pid_t* pid)
{
  return pid->target_val;    // 设置当前的目标值
}

/**
  * @brief  设置比例、积分、微分系数
  * @param  p：比例系数 P
  * @param  i：积分系数 i
  * @param  d：微分系数 d
	*	@note 	无
  * @retval 无
  */
void set_p_i_d(pid_t* pid,float p, float i, float d)
{
  pid->Kp = p;    // 设置比例系数 P
  pid->Ki = i;    // 设置积分系数 I
  pid->Kd = d;    // 设置微分系数 D
}

/**
  * @brief  PID算法实现
  * @param  actual_val:实际值
	*	@note 	无
  * @retval 通过PID计算后的输出
  */
float PID_realize(pid_t* pid,float actual_val)
{
  /*计算目标值与实际值的误差*/
  pid->err = pid->target_val + pid->target_dif_val- actual_val;

	    /* 限定闭环死区 */
    if((pid->err >= -pid->dead_space) && (pid->err <= pid->dead_space))
    {
        pid->err = 0;
        //pid->integral = 0;
    }
    
    /* 积分分离，偏差较大时去掉积分作用 */
    if (pid->err > -pid->int_separation && pid->err < pid->int_separation)
    {
        pid->integral += pid->err;    // 误差累积
        
        /* 限定积分范围，防止积分饱和 */
        if (pid->integral > pid->integral_max) 
            pid->integral = pid->integral_max;
        else if (pid->integral < -pid->integral_max) 
            pid->integral = -pid->integral_max;
    }
//		else 
//		{
//			pid->integral = 0;
//		}
	
  /*PID算法实现*/
  pid->actual_val = pid->Kp * pid->err + 
                   pid->Ki * pid->integral + 
                   pid->Kd * (pid->err - pid->err_last);

  /*误差传递*/
  pid->err_last = pid->err;

  /*返回当前实际值*/
  return pid->actual_val;
}
/**
  * @brief  定时器周期调用函数
  * @param  无
	*@note 	无
  * @retval 无
  */
float set_point=0.0; //目标值
int pid_status=0;    

#include "pwm.h"
#include "encoder.h"
#include "Moter_ZL.h"
#include "Oled.h"
#include "HWT101CT_TTL.h"
#include "motor.h"
#include "usart.h"  
#include "velocityProfile.h"
extern TrapeVelprofile_t tp_x,tp_y;//速度规划结构体
extern volatile float speed_dir_x ;
extern volatile float speed_dir_y ;
extern volatile uint8_t x_speed_plan_flag ;//1速度规划完，0执行完，或者静止状态
extern volatile uint8_t y_speed_plan_flag ;
volatile uint8_t x_set_speed_flag ;
volatile uint8_t y_set_speed_flag ;
volatile uint8_t w_set_flag = 0;
volatile float ti = 0;
float temp_err,temp_yaw;//2025.9.20添加
extern void time_period_fun()
{
	encoder_count_get();
	//my_car.yaw = angle;
	ti += ENCODER_TIME_S;//pid时间单位s
	if(ti >= tp_x.t) x_speed_plan_flag = 0;		//x轴达到目标脉冲数 标志清零
	if(ti >= tp_y.t) y_speed_plan_flag = 0;		//y轴达到目标脉冲数 标志清零
	if(x_speed_plan_flag)my_car.v_x = speed_dir_x * calcTrapezoidalVel(&tp_x, ti);//计算当前时刻目标速度？
	else if(!x_set_speed_flag) my_car.v_x = 0;
	if(y_speed_plan_flag)my_car.v_y = speed_dir_y * calcTrapezoidalVel(&tp_y, ti);
	else if(!y_set_speed_flag) my_car.v_y = 0;
	speed_translation(&my_car.motor_1);speed_translation(&my_car.motor_2);speed_translation(&my_car.motor_3);speed_translation(&my_car.motor_4);//计算轮速
	RobotCalculate();//麦轮正运动学解算--轮子编码值->底盘三轴里程计坐标
	if(!w_set_flag)car_yaw_pid();//角度闭环pid
	mecanum(my_car.v_y,my_car.v_x,my_car.w);//麦克纳姆轮逆运动学（xy坐标系）
	

	if(!my_car.stop_flag)
	{
		motor_pid(&my_car.motor_1);motor_pid(&my_car.motor_2);
		motor_pid(&my_car.motor_3);
		motor_pid(&my_car.motor_4);//速度闭环pid
		motor_control(&my_car.motor_1);motor_control(&my_car.motor_2);
		motor_control(&my_car.motor_3);
		motor_control(&my_car.motor_4);//PWM控制电机
	
	}
	else
	{		
		my_car.motor_1.PWM = 0;my_car.motor_2.PWM = 0;my_car.motor_3.PWM = 0;my_car.motor_4.PWM = 0;
		motor_control(&my_car.motor_1);motor_control(&my_car.motor_2);motor_control(&my_car.motor_3);motor_control(&my_car.motor_4);//PWM控制电机
	}
	
	
	/**********************/    
//	char buffer1[50],buffer2[50],buffer3[50],buffer4[50],buffer5[50],buffer6[50],buffer7[50],buffer8[50],buffer9[50]; // 确保缓冲区足够大
//	char buffer4[80];
//	//    sprintf(buffer1, "%f", my_car.v_x);
////    sprintf(buffer2, "%f", my_car.now_v_x);
////    sprintf(buffer3, "%f", my_car.motor_1.s_pid.i_out);
//    sprintf(buffer4, "%f,%f\r\n", my_car.motor_3.speed, my_car.motor_4.speed);
//    sprintf(buffer5, "%f", my_car.motor_1.speed);
//	sprintf(buffer6, "%f", temp_yaw);
//	sprintf(buffer7, "%f", my_car.yaw);
//	
////	Usart_SendString(UART5, buffer1);	
////	Usart_SendString(UART5, ",");	
////	Usart_SendString(UART5, buffer2);	
////	Usart_SendString(UART5, ",");	
////	Usart_SendString(UART5, buffer3);
////	Usart_SendString(UART5, ",");
//	Usart_SendString(UART5, buffer4);	
//	Usart_SendString(UART5, ",");	
//	Usart_SendString(UART5, buffer5);
//    Usart_SendString(UART5, ",");	
//	Usart_SendString(UART5, buffer6);
//    Usart_SendString(UART5, ",");	
//	Usart_SendString(UART5, buffer7);	
//	Usart_SendString(UART5, "\r\n");	
//	
//	/**********************/
	
}







