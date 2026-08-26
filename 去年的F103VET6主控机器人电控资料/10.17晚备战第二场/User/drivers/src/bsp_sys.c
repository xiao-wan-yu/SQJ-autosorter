#include "bsp_sys.h"

//自己加的
#include "motor.h"
#include "openMV.h"
#include "LobotServoController.h"
#include "vl53l1x.h"

void key_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;//初始化结构体
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;//根据需要选择输出模式
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;//输出引脚号
	GPIO_Init(GPIOE,&GPIO_InitStruct);
}
//void huidu_Config(void)
//{
//	GPIO_InitTypeDef GPIO_InitStruct;//初始化结构体
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;//根据需要选择输出模式
//	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;//输出引脚号
//	GPIO_Init(GPIOE,&GPIO_InitStruct);
//}
extern volatile uint8_t w_set_flag;
extern volatile uint8_t x_set_speed_flag ;
extern volatile uint8_t y_set_speed_flag ;
extern void go_to_xy(float x_distance,float x_speed_plan,float y_distance,float y_speed_plan,float a_x,float a_y);
#include "motor.h"
#include "usart.h"  
#include "velocityProfile.h"
#include "HWT101CT_TTL.h"
#include "delay.h"
static float absf(float a)
{
	return a>=0?a:-a;
}
void write_cri()
{
	float the_angle = angle;
	w_set_flag = 1;
	x_set_speed_flag = 1;
	my_car.w = 0.5;
	my_car.v_x =18;   //18
	//delay_ms(500);/*2025.8.30暂时注释 看能否缓解忽略转圈现象*/
	delay_ms(1000);
	
	MVData[0]=0x00;
	int i = 0;
	
	while(1)
	{
		if(MVData[0] == 0x62){
//			while(1)
//			{
//				float dis = read_dis1();
//				if(abs((int)dis-186)<5)break;
//				else{
//				go_to_xy(0,90,(dis-186)/15,90,30,60);
//				}
//			}
			MVData[0] = 0x00;
			i++;
			delay_ms(200);					//视觉拍球太早，晚一点点拍
			runActionGroup(77, 1);
		}
//		if(i == 3){
//			runActionGroup(210,1);//立桩机械臂抬起，防止触球
//			delay_ms(1500);
//		}
		if(absf(angle-the_angle)<5)break;
		float temp = absf(angle-the_angle);
//		if(temp>150&&temp<200)my_car.v_y =0.1;
//		else my_car.v_y =0;
	}
	w_set_flag = 0;
	x_set_speed_flag = 0;
	my_car.w = 0;
	my_car.v_x =0;
	my_car.target_yaw = angle;
}
//GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13);//IO电平读取，高或者低

