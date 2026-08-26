#ifndef _motor_h
#define _motor_h
#include "stm32f10x.h"

//弧度制转角度制
#define rad2dev(rad)  ( rad * 180.0/Pi)

//角度制转弧度制
#define dev2rad(dev)  (dev*Pi/180.0)

#define MECANUM_CAR_length  	21.0f //麦轮车车长(cm) (前进方向的轮距)
#define MECANUM_CAR_width    	31.0f //麦轮车车宽
#define WHEEL_DIAMETE      		6.5f      //麦轮直径 (cm)
#define Pi  									3.14159265359f                  //圆周率

#define ENCODER_TIME_S     		0.010f   //编码器计数时间  (s)
#define ENCODER_TIME_MS     	10.0f     //编码器计数时间  (ms)
#define ENCODER_ACCURACY 			896.0f  //编码器精度 (线数)

#define ENCODER_GEAR     			1.0f      //连接编码器的齿轮齿数
#define WHEEL_GEAR       			1.0f     //连接车轮的齿轮齿数
#define Tooth_Proportion  		ENCODER_GEAR/WHEEL_GEAR	//编码器齿数与车模齿数的比例大小
#define Perimeter 						Pi*WHEEL_DIAMETE							//轮子旋转一周产生的路程/cm

#define MY_CURRENT_MAX				(3.0f)//电流限幅
#define MY_PWM_MAX						(1000)//PWM限幅（勿改！！！）


#define HAVE_PID_INTEGRAL
//--------------------------------------------------------------------------------------------位置式PID
typedef struct{
  float Kp;
  float Ki;
  float Kd;
#ifdef HAVE_PID_INTEGRAL
  int index;            // 积分分离系数
  float Integral;       // 积分项
  float I_outputMax;    // 积分限幅
#endif
	float Err;
  float Last_Err;       // 上次误差
  float Output;         // PID输出
  float OutputMax;      // 位置式PID输出限幅
}Position_PID;
//--------------------------------------------------------------------------------------------增量式PID
typedef struct{
  float Kp;
  float Ki;
  float Kd;
  float p_out;
  float i_out;
  float d_out;
  float Err;
  float Last_Err;       // 上次误差
  float Previous_Err;   // 上上次误差
  float Output;
  float OutputMax;      // 增量式式PID输出限幅
}Incremental_PID;


//--------------------------------------------------------------------------------------------电机参数
typedef struct {
	
	//电机初始化参数
	GPIO_TypeDef* dir_GPIOx; //方向引脚
	uint16_t dir_GPIO_Pin;
	uint8_t pwm_ch;					//PWM引脚
	uint16_t current_pin;			//电流adc引脚
	int8_t motor_dir;					//电机极性	
	
	
	volatile float target_speed;		//目标速度（cm/s）
	volatile float speed;						//实时速度（cm/s）
	volatile float current;					//实时电流
	int32_t PWM;											//PWM值
	
	Incremental_PID s_pid;						//速度环pid
	Incremental_PID c_pid;						//电流环pid
	
	//编码器初始化参数
	uint16_t A_pin;						//A相引脚
	uint16_t B_pin;						//B相引脚
	int8_t encoder_dir;				//编码器极性
	
	int32_t encoder_count_all_last;				//编码器累计值
	int32_t encoder_count_all;				//编码器累计值
	int32_t encoder_count;						//编码器计数值(5ms内)
	int32_t encoder_count_r;					//编码器计数值(纠正极性)
	float	 encoder_count_f;					//编码器单位时间计数值（滤波）

	float pressure;									//轮子受到的压力（kg）
}Motor;


//--------------------------------------------------------------------------------------------小车参数
typedef struct{
	
	Motor motor_1,motor_2,motor_3,motor_4;//右前，左前，左后，右后
	
	volatile float target_yaw;		//目标角度
	
	volatile float speed;					//整车目标速度
	volatile float the;						//移动方向（向右为0°）
	volatile float w;							//目标角速度
	volatile float v_x;						//横移速度（向右为正）
	volatile float v_y;						//前进速度（向前为正）
	double target_point[2];				//目标坐标点(x,y)
	
	
	volatile float now_v_x;						//当前横移速度（向右为正）
	volatile float now_v_y;						//当前前进速度（向前为正）
	volatile float yaw;							//当前角度
	volatile double now_point[2];		//当前坐标点(x,y)
	volatile double now_x;					//当前x坐标
	volatile double now_y;					//当前y坐标
	volatile float now_the;				
	Position_PID yaw_pid;						//转向环pid
	Position_PID x_pid,y_pid;				//位置环pid
	
	uint8_t stop_flag;
}Car;


//外部变量声明
extern Car my_car;

void PositionPID_clear(Position_PID *pid);
void motor_adc_init(void);
void stop_car(void);//停车
void RobotCalculate(void);//麦轮正运动学解算
void mecanum(double v_y,double  v_x  ,double  w);//麦轮逆运动学解算（xy坐标系）
void mecanum_polar( double v , double the ,double w);//麦轮逆运动学解算（笛卡尔坐标系）
void car_yaw_pid(void);//车辆角度环pid计算
void current_get(void);//获取各个电机电流值
void motor_pid(Motor *motor);//电机速度环pid计算
void PositionPID_Calculate(Position_PID *pid,const float Target,const float Measure);//位置式pid
void IncrementalPID_Calculate(Incremental_PID *pid,const float Target,const float Measure);//增量式pid
void speed_translation(Motor *motor);//轮子速度换算
void encoder_count_get(void);//获取编码器值
void car_init(void);//整车参数初始化
void encoder_A_exti_handler(Motor *motor);//编码器A相中断服务程序
void encoder_B_exti_handler(Motor *motor);//编码器B相中断服务程序
void motor_control(Motor *motor);//pwm控制电机

//-------------------------------------------------------------------------------------------------------------------
//  @brief      输入x方向速度(cm/s)  y方向速度(cm/s) 旋转速度  (  °/s )
//  @param      encoder_data      编码器测量值
//  @param      target_encoder    目标编码器值(希望 逼近值)
//  @param      motor_num         电机编号  (左上为1  右上为2  右下为3   左下为4 )
//  @return     void
//  Sample usage:           deviation(encoder_data[0] ,target_encoder[0] ,1 )
//-------------------------------------------------------------------------------------------------------------------
void  mecanum(double v_y,double  v_x  ,double  w);


//-------------------------------------------------------------------------------------------------------------------
//  @brief      输入行走速度v, 行走方向the (°)   旋转速度  w( °/s  )  (极坐标系形式)
//  @param      v        v轴方向速度   (cm/s)
//  @param      the      行走方向the ( 0为右移, 90为前进  ,180为左移 , 270°为 后退 , 360 为 右移  大于360则会对其进行取模
//  @param      w        旋转方向        (逆时针为 正)  (rad/s )
//  @return     void
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void  mecanum_polar( double v , double the ,double w);
#endif
