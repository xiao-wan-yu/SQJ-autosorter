#include "motor.h"
#include "math.h"
#include "HWT101CT_TTL.h"
#include "usart.h"  

//#define speed_p 4.0f//5.0f
//#define speed_i 1.0f//1.2f
//#define speed_d 0.0f//0.0f

//#define speed_p 4.5f//5.0f    //////2025.9.20合适参数（前后响应稍微迟钝）
//#define speed_i 1.3f//1.2f
//#define speed_d 0.0f//0.0f

//#define speed_p 4.5f//9.5峰值可达到目标值，但是一直在下方震荡；7.0以下较平稳，7.5震荡
//#define speed_i 1.3f//10.0
//#define speed_d 0.0f//

//#define speed_p 5.2f//5.8
//#define speed_i 0.03f//0.1      2025.9.20下午整定9.28还在用
//#define speed_d 0.0f//

#define speed_p 5.2f//5.8
#define speed_i 0.03f//0.1      
#define speed_d 0.0f//

//#define yaw_p -0.05f//0.08f//0.05f
//#define yaw_i -0.01f//0.02f//0.01
//#define yaw_d -0.0f//1.2f

//#define yaw_p -0.03f//2025.9.20下午整定
//#define yaw_i -0.05f//
//#define yaw_d -0.0f//

//#define yaw_p -0.08f//2025.9.20下午整定(快且还行)
//#define yaw_i -0.08f//
//#define yaw_d -0.0f//

//#define yaw_p -0.017f//
//#define yaw_i -0.032f//
//#define yaw_d -0.0f//


//#define yaw_p -0.01f//0.01
//#define yaw_i -0.0f//		2025.9.20下午整定9.28还在用
//#define yaw_d -0.08f//0.08

#define yaw_p -0.02f//
#define yaw_i -0.0f//		
#define yaw_d -0.02f//0.08


#define yaw_p_k	0.008f
#define yaw_d_k	0.008f

#define x_p 0//80.0f
#define x_i 0//10.5f
#define x_d 0//220.6f

#define y_p 0//5.0f
#define y_i 0//2.0f
#define y_d 0//100.0f

float abs_f(float num)
{
	return (num>=0) ? num:-num;
}

float limit(float a,float max)
{
	if(a>max) return max;
	else if(a<-max) return -max;
	else return a;
}

void motor_adc_init(void)
{
	//电流检测引脚初始化
//	adc_init(my_car.motor_1.current_pin, ADC_12BIT); 
//	adc_init(my_car.motor_2.current_pin, ADC_12BIT); 
//	adc_init(my_car.motor_3.current_pin, ADC_12BIT); 
//	adc_init(my_car.motor_4.current_pin, ADC_12BIT); 
}

Car my_car;
void car_init(void)//整车参数初始化	右前	左前	左后	右后
{
	my_car.stop_flag = 0;
	
//	my_car.motor_1.pressure = 1.6f;//轮子承受的压力
//	my_car.motor_2.pressure = 1.32f;
//	my_car.motor_3.pressure = 1.1f;
//	my_car.motor_4.pressure = 0.85f;
	
	my_car.motor_1.pressure = 0.1f;//轮子承受的压力
	my_car.motor_2.pressure = 0.1f;
	my_car.motor_3.pressure = 0.1f;
	my_car.motor_4.pressure = 0.1f;
	
//	//*********************************************************电机1
//	//引脚定义
//	my_car.motor_1.dir_GPIOx = GPIOD;	
//	my_car.motor_1.dir_GPIO_Pin = GPIO_Pin_3;
//	my_car.motor_1.pwm_ch = 1;

//	//参数初始化
//	my_car.motor_1.motor_dir = -1;
//	my_car.motor_1.encoder_dir = -1;
//	my_car.motor_1.PWM = 0;
//	my_car.motor_1.target_speed = 0.0f;
//	
//	//*********************************************************电机2
//	//引脚定义
//	my_car.motor_2.dir_GPIOx = GPIOD;	
//	my_car.motor_2.dir_GPIO_Pin = GPIO_Pin_4;
//	my_car.motor_2.pwm_ch = 2;
//	//参数初始化
//	my_car.motor_2.motor_dir = 1;
//	my_car.motor_2.encoder_dir = 1;
//	my_car.motor_2.PWM = 0;
//	my_car.motor_2.target_speed = 0.0f;

//	//*********************************************************电机3
//	//引脚定义
//	my_car.motor_3.dir_GPIOx = GPIOD;	
//	my_car.motor_3.dir_GPIO_Pin = GPIO_Pin_5;
//	my_car.motor_3.pwm_ch = 3;
//	//参数初始化
//	my_car.motor_3.motor_dir = -1;
//	my_car.motor_3.encoder_dir = -1;
//	my_car.motor_3.PWM = 0;
//	my_car.motor_3.target_speed = 0.0f;

//	//*********************************************************电机4
//	//引脚定义
//	my_car.motor_4.dir_GPIOx = GPIOD;	
//	my_car.motor_4.dir_GPIO_Pin = GPIO_Pin_6;
//	my_car.motor_4.pwm_ch = 4;
//	//参数初始化
//	my_car.motor_4.motor_dir = 1;
//	my_car.motor_4.encoder_dir = 1;
//	my_car.motor_4.PWM = 0;
//	my_car.motor_4.target_speed = 0.0f;

/*******自己加的**********/
		//*********************************************************电机1
	//引脚定义
	my_car.motor_1.dir_GPIOx = GPIOD;	
	my_car.motor_1.dir_GPIO_Pin = GPIO_Pin_3;
	my_car.motor_1.pwm_ch = 1;

	//参数初始化
	my_car.motor_1.motor_dir = 1;
	my_car.motor_1.encoder_dir = -1;
	my_car.motor_1.PWM = 0;
	my_car.motor_1.target_speed = 0.0f;
	
	//*********************************************************电机2
	//引脚定义
	my_car.motor_2.dir_GPIOx = GPIOD;	
	my_car.motor_2.dir_GPIO_Pin = GPIO_Pin_4;
	my_car.motor_2.pwm_ch = 2;
	//参数初始化
	my_car.motor_2.motor_dir = -1;
	my_car.motor_2.encoder_dir = 1;
	my_car.motor_2.PWM = 0;
	my_car.motor_2.target_speed = 0.0f;

	//*********************************************************电机3
	//引脚定义
	my_car.motor_3.dir_GPIOx = GPIOD;	
	my_car.motor_3.dir_GPIO_Pin = GPIO_Pin_5;
	my_car.motor_3.pwm_ch = 3;
	//参数初始化
	my_car.motor_3.motor_dir = 1;
	my_car.motor_3.encoder_dir = -1;
	my_car.motor_3.PWM = 0;
	my_car.motor_3.target_speed = 0.0f;

	//*********************************************************电机4
	//引脚定义
	my_car.motor_4.dir_GPIOx = GPIOD;	
	my_car.motor_4.dir_GPIO_Pin = GPIO_Pin_6;
	my_car.motor_4.pwm_ch = 4;
	//参数初始化
	my_car.motor_4.motor_dir = -1;
	my_car.motor_4.encoder_dir = 1;
	my_car.motor_4.PWM = 0;
	my_car.motor_4.target_speed = 0.0f;

/*******自己加的**********/
	
	//速度环PID参数初始化（增量式PI控制）
	my_car.motor_1.s_pid.Kp = speed_p;
	my_car.motor_1.s_pid.Ki = speed_i;
	my_car.motor_1.s_pid.Kd = speed_d;
	my_car.motor_1.s_pid.OutputMax = MY_PWM_MAX;
	
	my_car.motor_2.s_pid.Kp = speed_p;
	my_car.motor_2.s_pid.Ki = speed_i;
	my_car.motor_2.s_pid.Kd = speed_d;
	my_car.motor_2.s_pid.OutputMax = MY_PWM_MAX;
	
	my_car.motor_3.s_pid.Kp = speed_p;
	my_car.motor_3.s_pid.Ki = speed_i;
	my_car.motor_3.s_pid.Kd = speed_d;
	my_car.motor_3.s_pid.OutputMax = MY_PWM_MAX;
	
	my_car.motor_4.s_pid.Kp = speed_p;
	my_car.motor_4.s_pid.Ki = speed_i;
	my_car.motor_4.s_pid.Kd = speed_d;
	my_car.motor_4.s_pid.OutputMax = MY_PWM_MAX;
	
	
	//角度环PID参数初始化（位置式PID控制）
	my_car.yaw_pid.Kp = yaw_p;
	my_car.yaw_pid.Ki = yaw_i;
	my_car.yaw_pid.Kd = yaw_d;
	my_car.yaw_pid.OutputMax = 2.8f;//2.5f
	
//	//位置环PID参数初始化
//	my_car.x_pid.Kp = x_p;
//	my_car.x_pid.Ki = x_i;
//	my_car.x_pid.Kd =	x_d;
//	my_car.x_pid.OutputMax = 0.0f;
//	
//	my_car.y_pid.Kp = y_p;
//	my_car.y_pid.Ki = y_i;
//	my_car.y_pid.Kd = y_d;
//	my_car.y_pid.OutputMax = 0.0f;
	
	//整车参数初始化
	my_car.the = 90.0f;
	my_car.speed = 0.0f;
	my_car.w = 0.0f;
	my_car.v_x = 0.0f;
	my_car.v_y = 0.0f;
	my_car.target_yaw = 0.0f;
	
	//初始位置坐标
	my_car.now_x = 0.0f;
	my_car.now_y = 0.0f;
	
	my_car.now_point[0] = my_car.now_x;
	my_car.now_point[1] = my_car.now_y;	
	
	my_car.target_point[0] = my_car.now_point[0];
	my_car.target_point[1] = my_car.now_point[1];	
	
}

//--------------------------------------------------------------------------------------------位置式PID
#define Integraldead_zone 0.1 // 积分死区 根据自己的需求定义
//#define Integraldead_zone 10.0 //积分死区 根据自己的需求定义

float NoWay = 0.005;    //还抖，没门！2025.9.28    0.0025-0.01多还行  0.005对应p是0.01
//每次转角度前后都要关闭和打开，否则会卡在

void PositionPID_Calculate(Position_PID *pid,const float Target,const float Measure)
{

//  if(!*((char*)&pid))			//此处没什么用2025.9.29注释
//    return;
	pid->Err = Target - Measure;
	
  pid->Output = pid->Kp * pid->Err + pid->Kd * (pid->Err - pid->Last_Err);
    if(pid->Output > -NoWay && pid->Output < NoWay) pid->Output = 0 ;//要注意负数
  
#ifdef HAVE_PID_INTEGRAL
    /* 积分分离 */
  if(abs_f(pid->Err) < Integraldead_zone)
  {
    pid->index=0;
  }else
  {
    pid->index = 1;
  }
  pid->Integral += pid->Ki * pid->Err * pid->index;
  
  pid->Integral = limit(pid->Integral,pid->I_outputMax);
  
  pid->Output += pid->Integral;
#endif
  
  pid->Output = limit(pid->Output,pid->OutputMax);
  
  pid->Last_Err = pid->Err;
  
}


//--------------------------------------------------------------------------------------------增量式PID
void IncrementalPID_Calculate(Incremental_PID *pid,const float Target,const float Measure)
{ 
//  if(!*((char*)&pid))				//此处没什么用2025.9.29注释
//    return;
  pid->Err = Target - Measure;	
	
  pid->p_out = pid->Kp * (pid->Err - pid->Last_Err);
  pid->d_out = pid->Kd * (pid->Err - 2.0f*pid->Last_Err + pid->Previous_Err);
  pid->i_out += pid->Ki * pid->Err;
//  if((pid->p_out+pid->d_out <= 300)&&(pid->p_out+pid->d_out >= -300))
//	  pid->i_out = pid->Ki * pid->Err;//2025.9.19改动 改成了条件积分宣告失败
  if(pid->i_out > 10) pid->i_out = 10;          //2025.9.20
  if(pid->i_out < -10) pid->i_out = -10;
  if( pid->Ki * pid->Err >-0.1 &&  pid->Ki * pid->Err<0.1){
	if(pid->i_out > 0.2)pid->i_out = 0.2;
	else if(pid->i_out <-0.2) pid->i_out = -0.2;
//	if(pid->i_out > 0 && pid->i_out < 1.0)pid->i_out = 0;
//    if(pid->i_out < 0 && pid->i_out > -1.0) pid->i_out = 0;
  }
  
  pid->Output += pid->p_out + pid->i_out + pid->d_out; 
  
  pid->Output = limit(pid->Output, pid->OutputMax); // 限幅
  
  pid->Previous_Err = pid->Last_Err;
  pid->Last_Err = pid->Err;
  
}

//清空累计值
void PositionPID_clear(Position_PID *pid)
{
	pid->Err = 0;
  pid->Last_Err = 0;       // 上次误差
  pid->Output = 0;         // PID输出
}



#define k_p 1.0f
//车长的一半
#define HALF_LENGTH  (MECANUM_CAR_length/2.0f)  //  18/2 =  9
//车宽的一半
#define HALF_WIDTH   (MECANUM_CAR_width/2.0f)    //  18/2 =  9

//麦克娜姆轮逆运动学建模  公式参考:https://blog.csdn.net/weixin_30627381/article/details/97069120
//-------------------------------------------------------------------------------------------------------------------
//  @brief      输入x方向速度(cm/s)  y方向速度(cm/s) 旋转速度  (  rad/s )  求解得到目标编码器计数值
//  @param      v_y      y轴方向速度 (前进为正)  (cm/s)
//  @param      v_x      x轴方向速度(向右运动为正 (cm/s)
//  @param      w        旋转方向(逆时针为 正)  (rad/s )
//  @return     void
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------

void  mecanum(double v_y,double  v_x  ,double  w)
{
//左上右上左下右下分别为电机1234
		my_car.motor_1.target_speed = ( v_y + v_x - w* (HALF_LENGTH +  HALF_WIDTH));
		my_car.motor_2.target_speed = ( v_y - v_x + w* (HALF_LENGTH +  HALF_WIDTH));
		my_car.motor_3.target_speed = ( v_y - v_x - w* (HALF_LENGTH +  HALF_WIDTH));
		my_car.motor_4.target_speed = ( v_y + v_x + w* (HALF_LENGTH +  HALF_WIDTH));

		
//		limit(my_car.motor_1.target_speed, 200.0f); // 限幅
//		limit(my_car.motor_2.target_speed, 200.0f); // 限幅
//		limit(my_car.motor_3.target_speed, 200.0f); // 限幅
//		limit(my_car.motor_4.target_speed, 200.0f); // 限幅
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      输入行走速度v, 行走方向the (°)   旋转速度  w( °/s  )  (极坐标系形式)  求解得到目标编码器计数值
//  @param      v        v轴方向速度   (cm/s)
//  @param      the      行走方向the ( 0为右移, 90为前进  ,180为左移 , 270°为 后退 , 360 为 右移  大于360则会对其进行取模
//  @param      w        旋转方向        (逆时针为 正)  (rad/s )
//  @return     void
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------

void  mecanum_polar( double v , double the ,double w)
{
    double  v_y ,v_x  , rad , the_mod, w_rad;
    the_mod = fmod( the , 360.0) ;//浮点数取模
    rad =  dev2rad(the_mod)  ; //角度值变弧度制
    v_x = v * cos(rad);
    v_y = v * sin(rad);
    w_rad = dev2rad(w);//角度值变弧度制
    mecanum( v_y , v_x  , w_rad) ; //填入笛卡儿坐标形式运动学建模
}


/**
  * @函数功能：正向运动学解析，轮子编码值->底盘三轴里程计坐标
  */
void RobotCalculate(void)
{
    // 根据每帧的motors[4]计算更新g_robot
    float deltacounts[4]={0,0,0,0};
    float delta_x_o, delta_y_o;
    float delta_x, delta_y;
    float everycount;
    
    my_car.now_the = (360-my_car.yaw) * Pi / 180.0f;
    
    while((my_car.now_the >= 2.0f * Pi) || (my_car.now_the < 0))
    {
        if(my_car.now_the >= 2.0f * Pi)
            my_car.now_the = my_car.now_the - 2.0f * Pi;
        if(my_car.now_the < 0)
            my_car.now_the = my_car.now_the + 2.0f * Pi;
    }
    
    everycount = ((1.0f/ENCODER_ACCURACY)*Tooth_Proportion)*Perimeter;//每个脉冲走多少cm
    
    deltacounts[0] = my_car.motor_1.encoder_count_f;
    deltacounts[1] = my_car.motor_2.encoder_count_f;
    deltacounts[2] = my_car.motor_4.encoder_count_f;
    deltacounts[3] = my_car.motor_3.encoder_count_f;
        
    delta_x_o = (-deltacounts[1] + deltacounts[2]) / (2.0f) * everycount;
    delta_y_o = (deltacounts[0] + deltacounts[1]) / (2.0f) * everycount;


    if (my_car.now_the >= 0 && my_car.now_the < (Pi/2.0f))
    {
        delta_x = delta_x_o * sin(Pi/2.0f - my_car.now_the) - delta_y_o * sin(my_car.now_the);
        delta_y = delta_x_o * cos(Pi/2.0f - my_car.now_the) + delta_y_o * cos(my_car.now_the);
    }
    
    if (my_car.now_the >= (Pi/2.0f) && my_car.now_the < Pi)
    {
        delta_x = -(delta_x_o * sin(my_car.now_the - Pi/2.0f) + delta_y_o * cos(my_car.now_the - (Pi/2.0f)));
        delta_y = delta_x_o * cos(my_car.now_the - Pi/2.0f) - delta_y_o * sin(my_car.now_the - (Pi/2.0f));
    }
    
    if (my_car.now_the >= Pi && my_car.now_the < (3.0f*Pi/2.0f))
    {
        delta_x = -delta_x_o * cos(my_car.now_the - Pi) + delta_y_o * cos(3.0f*Pi/2.0f - my_car.now_the);
        delta_y = -(delta_x_o * sin(my_car.now_the - Pi) + delta_y_o * sin(3.0f*Pi/2.0f - my_car.now_the));
    }
    
    if (my_car.now_the >= (3.0f*Pi/2.0f) && my_car.now_the < (2.0f*Pi))
    {
        delta_x = delta_x_o * sin(my_car.now_the - 3.0f*Pi/2.0f) + delta_y_o * cos(my_car.now_the - 3.0f*Pi/2.0f);
        delta_y = -delta_x_o * cos(my_car.now_the - 3.0f*Pi/2.0f) + delta_y_o * sin(my_car.now_the - 3.0f*Pi/2.0f);
    }
    
		my_car.now_v_x = delta_x / 0.005f;
		my_car.now_v_y = delta_y / 0.005f;
    my_car.now_x += delta_x;
    my_car.now_y += delta_y;
		my_car.now_point[0] = my_car.now_x;
		my_car.now_point[1] = my_car.now_y;

}



void car_yaw_pid(void)//角度闭环pid
{
//	//角度环PID参数初始化（位置式PID控制）
//	my_car.yaw_pid.Kp = yaw_p + yaw_p_k * fabs(my_car.v_x);
//	my_car.yaw_pid.Ki = yaw_i;
//	my_car.yaw_pid.Kd = yaw_d + yaw_d_k * fabs(my_car.v_x);
//	my_car.yaw_pid.OutputMax = 20.0f;
	
//	float temp_err,temp_yaw;
//	temp_err = my_car.target_yaw - my_car.yaw;
//		//取最短路径
//	if(temp_err > 180.0f)
//	{
//	  temp_err = -360.0f + temp_err;
//	}

//	else if(temp_err < -180.0f)
//	{
//	  temp_err = 360.0f + temp_err;
//	}
//	
//	else if(fabs(temp_err) <= 180.f)
//	{
//		temp_err = temp_err;
//	}
//	
//	temp_yaw = temp_err + my_car.yaw;
//	
//	my_car.yaw = angle;
	extern float temp_err,temp_yaw;
	my_car.yaw = angle;
	temp_err = my_car.target_yaw - my_car.yaw;
		//取最短路径
	if(temp_err > 180.0f)
	{
	  temp_err = -360.0f + temp_err;
	}

	else if(temp_err < -180.0f)
	{
	  temp_err = 360.0f + temp_err;
	}
	
	else if(fabs(temp_err) <= 180.f)
	{
		temp_err = temp_err;
	}
	
	temp_yaw = temp_err + my_car.yaw;
	
	
	PositionPID_Calculate(&my_car.yaw_pid,temp_yaw,my_car.yaw);
	my_car.w = my_car.yaw_pid.Output;
}

void motor_pid(Motor *motor)//电机速度环pid计算
{
	IncrementalPID_Calculate(&motor->s_pid,motor->target_speed,motor->speed);
	motor->PWM = motor->s_pid.Output;
	//printf("%f-%f-%f\n",motor->s_pid.Output,motor->target_speed,motor->speed);
}


/************************ 滑动窗口滤波器 *****************************/
//#define window_size 5            //滑动窗口长度
//float buffer_1[window_size] = {0}; //滑动窗口数据buf
//float buffer_2[window_size] = {0}; //滑动窗口数据buf
//float buffer_3[window_size] = {0}; //滑动窗口数据buf
//float buffer_4[window_size] = {0}; //滑动窗口数据buf
/*********************** 滑动窗口滤波函数 ****************************/
float sliding_average_filter(float value,float buffer[])
{
//  static int data_num = 0;
//  float output = 0;

//  if (data_num < window_size) //不满窗口，先填充
//  {
//    buffer[data_num++] = value;
//    output = value; //返回相同的值
//  }
//  else
//  {
//    int i = 0;
//    float sum = 0;

//    memcpy(&buffer[0], &buffer[1], (window_size - 1) * 4); //将1之后的数据移到0之后，即移除头部
//    buffer[window_size - 1] = value;                       //即添加尾部

//    for (i = 0; i < window_size; i++) //每一次都计算，可以避免累计浮点计算误差
//      sum += buffer[i];

//    output = sum / window_size;
//  }

//  return output;
}

#define current_k 3.3f/4096.0f/20.0f/0.050f

void current_get(void)//获取各个电机电流值
{
	
//	my_car.motor_1.current = ((volatile float)adc_mean_filter_convert(my_car.motor_1.current_pin,5))*current_k;
//	my_car.motor_2.current = ((volatile float)adc_mean_filter_convert(my_car.motor_2.current_pin,5))*current_k;
//	my_car.motor_3.current = ((volatile float)adc_mean_filter_convert(my_car.motor_3.current_pin,5))*current_k;
//	my_car.motor_4.current = ((volatile float)adc_mean_filter_convert(my_car.motor_4.current_pin,5))*current_k;
}
#include "encoder.h"
void encoder_count_get(void)//获取编码器值
{
	my_car.motor_1.encoder_count_all = encoder1_num*5120+TIM_GetCounter(TIM2);//读取编码器总计数值
	my_car.motor_2.encoder_count_all = encoder2_num*5120+TIM_GetCounter(TIM3);
	my_car.motor_3.encoder_count_all = encoder3_num*5120+TIM_GetCounter(TIM4);
	my_car.motor_4.encoder_count_all = encoder4_num*5120+TIM_GetCounter(TIM5);
	
	my_car.motor_1.encoder_count = my_car.motor_1.encoder_count_all - my_car.motor_1.encoder_count_all_last;//计算增加值
	my_car.motor_2.encoder_count = my_car.motor_2.encoder_count_all - my_car.motor_2.encoder_count_all_last;
	my_car.motor_3.encoder_count = my_car.motor_3.encoder_count_all - my_car.motor_3.encoder_count_all_last;
	my_car.motor_4.encoder_count = my_car.motor_4.encoder_count_all - my_car.motor_4.encoder_count_all_last;
	
	my_car.motor_1.encoder_count_all_last = my_car.motor_1.encoder_count_all;//更新上一次的值
	my_car.motor_2.encoder_count_all_last = my_car.motor_2.encoder_count_all;
	my_car.motor_3.encoder_count_all_last = my_car.motor_3.encoder_count_all;
	my_car.motor_4.encoder_count_all_last = my_car.motor_4.encoder_count_all;
	
	my_car.motor_1.encoder_count_r = my_car.motor_1.encoder_count * my_car.motor_1.encoder_dir;//极性校正
	my_car.motor_2.encoder_count_r = my_car.motor_2.encoder_count * my_car.motor_2.encoder_dir;
	my_car.motor_3.encoder_count_r = my_car.motor_3.encoder_count * my_car.motor_3.encoder_dir;
	my_car.motor_4.encoder_count_r = my_car.motor_4.encoder_count * my_car.motor_4.encoder_dir;
	
	my_car.motor_1.encoder_count_f =  my_car.motor_1.encoder_count_r;//滤波值
	my_car.motor_2.encoder_count_f =  my_car.motor_2.encoder_count_r;
	my_car.motor_3.encoder_count_f =  my_car.motor_3.encoder_count_r;
	my_car.motor_4.encoder_count_f =  my_car.motor_4.encoder_count_r;

}
#include "protocol.h"
void speed_translation(Motor *motor)//轮子速度换算g
{
	motor->speed = ((motor->encoder_count_f/ENCODER_ACCURACY)*Tooth_Proportion)*Perimeter/ENCODER_TIME_S;
	int teemp = (int)(motor->speed);
	//if(motor->pwm_ch==1)set_computer_value(SEND_FACT_CMD, CURVES_CH1, &teemp, 1);
	
	//printf("%f\n",motor->speed);
}




#define motor_dead	200
#include "pwm.h"

void motor_control(Motor *motor)//pwm控制电机
{
		
    //对占空比限幅
		motor->PWM = limit(motor->PWM, MY_PWM_MAX);
		
		int32_t duty;
		duty = motor->PWM * motor->motor_dir;//电机极性纠正

    if(duty >= 0)																					//正转
    {
			GPIO_SetBits(motor->dir_GPIOx, motor->dir_GPIO_Pin);// DIR输出高电平
			if(motor->pwm_ch==1)TIM_SetCompare1(TIM8,duty);
			else if(motor->pwm_ch==2)TIM_SetCompare2(TIM8,duty);
			else if(motor->pwm_ch==3)TIM_SetCompare3(TIM8,duty);
			else if(motor->pwm_ch==4)TIM_SetCompare4(TIM8,duty);	

    }
    else																									//反转
    {
			GPIO_ResetBits(motor->dir_GPIOx, motor->dir_GPIO_Pin);// DIR输出高电平
			if(motor->pwm_ch==1)TIM_SetCompare1(TIM8,-duty);
			else if(motor->pwm_ch==2)TIM_SetCompare2(TIM8,-duty);
			else if(motor->pwm_ch==3)TIM_SetCompare3(TIM8,-duty);
			else if(motor->pwm_ch==4)TIM_SetCompare4(TIM8,-duty);	

    }
}

void stop_car(void)
{
		my_car.v_y = 0.0f;
		my_car.v_x = 0.0f;
}