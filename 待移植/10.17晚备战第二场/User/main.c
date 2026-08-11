/********************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2018-xx-xx
  * @brief   FreeRTOS v9.0.0 + STM32 动态创建多任务
  ********************************************************************/ 
 
/*-------------------------------文件包含--------------------------------*/ 

//#include "FreeRTOS.h"                                                     /* FreeRTOS头文件 */
//#include "task.h"
#include "usart.h"                                                        /* 开发板硬件bsp头文件 */
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "Blue_HC05.h"
#include "openMV.h"
#include "8LuHuiDu.h"
#include "task1.h"
#include "task2.h"
#include "task3.h"
#include "task_timer.h"
#include "Moter.h"
#include "led.h"
#include "delay.h"
#include "timer.h"
#include "encoder.h"
#include "pwm.h"
#include "Moter_ZL.h"
#include "Oled.h"
#include "pid_timer.h"
#include "bsp_pid.h"
#include "protocol.h"
#include "HWT101CT_TTL.h"
#include "motor.h"
#include "velocityProfile.h"
//#include "bsp_adc.h"
#include "vl53l1x.h"
#include "LobotServoController.h"
#include "bsp_sys.h"
#include "Scanner.h"

TrapeVelprofile_t tp_x,tp_y;//速度规划结构体

extern volatile uint8_t x_speed_plan_flag ;//1速度规划完，0执行完，或者静止状态
extern volatile uint8_t y_speed_plan_flag ;
extern volatile uint8_t x_set_speed_flag ;
extern volatile uint8_t y_set_speed_flag ;
extern float temp_err,temp_yaw;//2025.9.20添加
extern volatile float NoWay;    //还抖，没门！2025.9.28    0.0025-0.01多还行
//每次转角度前后都要关闭和打开，否则会卡在转角度
char buffer4[80];


void go_to_xy(float x_distance,float x_speed_plan,float y_distance,float y_speed_plan,float a_x,float a_y);
void jieti_hong(void);
void jieti_Blue(void);
void WareHouse(void);
void LiZhuang(void);
void GoBack(void);
void cangkufangqiu(void);
void lizhuang(void);
void lizhuang222(void);
void lizhuang2221(void);
void jieti222(void);
void jieti2221(void);
void jieti2223(void);
int rb_t;//1红，2蓝
int main(void)
{	
	delay_ms(100);

/*-------------------------初始化--------------------------*/
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );                        //后期配置中断向量表时都不需要配置分组
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
//  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);                 //将PA15、PB3、PB4配置为普通IO口

	USART_Config();                                                          //串口1初始化，用于调试
//	Oled_Init();
	delay_ms(500);
	HWT101CT_init();

	TIM8_PWM_init();
	Encoder_Init_TIM2();//A15  B3
	Encoder_Init_TIM3();//B4  B5
	Encoder_Init_TIM4();//B6  B7
	Encoder_Init_TIM5();//A0  A1
	
	protocol_init();
	vl53l1x_init();
	USART4_Config();
	openMV_init();
	//openMV2_init();
	//speed_BasicTim_Init();
	car_init();
	PID_BasicTim_Init();

	float pid_temp[3] = {1, 0, 0};
    set_computer_value(SEND_P_I_D_CMD, CURVES_CH1, pid_temp, 3);     // 给通道 1 发送 P I D 值

	int t =100;
    set_computer_value(SEND_STOP_CMD, CURVES_CH1, NULL, 0);                // 同步上位机的启动按钮状态
    set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &t, 1);     // 给通道 1 发送目标值
	

	MVData2[0] = 'a';
	MVData2[1] = '\0';
	
//	u8g2_ClearBuffer(&u8g2);
//	u8g2_SetFont(&u8g2,u8g2_font_ncenB08_tf);
//	u8g2_DrawStr(&u8g2, 0,50,"Init OK");
//	u8g2_SendBuffer(&u8g2);
	
//	char buffer[100];	
//	while(1)
//	{
//		float dis = read_dis1();   
//		sprintf(buffer, "qian:%5.2f\r\n", dis);
//		Usart_SendString(UART5, buffer);
//	}

//runActionGroup(168, 1);		//确认红蓝方后所有机械臂复位
//delay_ms(5000);
//runActionGroup(4, 1);//机械臂抬起，该动作组运行时间为1秒
//delay_ms(1150);
//runActionGroup(5, 1);//机械臂抬起，该动作组运行时间为1秒
//delay_ms(2000);
//runActionGroup(4, 1);//机械臂抬起，该动作组运行时间为1秒
//delay_ms(1150);
//runActionGroup(10, 1);
//delay_ms(3000);
//while(1);


//	
	//********************调试区**********************//
//	goto start;
//char buffer[50];
//while(1){
//	sprintf(buffer, "%f\r\n", my_car.yaw);
//	Usart_SendString(UART5, buffer);
//}
	if(0){
////	my_car.target_yaw = 270;                     // 转角度
////	while(!(abs(10*(my_car.yaw-270))<5));        //等待转90完成
////	
////	while(1);
//	
////	delay_ms(3000);
////	Usart_SendByte(UART5, 0x31);
////	
////	u8g2_ClearBuffer(&u8g2);
////			u8g2_SetFont(&u8g2,u8g2_font_ncenB08_tf);
////			u8g2_DrawStr(&u8g2, 0,50,"Red");
////			u8g2_SendBuffer(&u8g2);
////	
////	Usart_SendByte(UART5, 0x33);//告诉mv开始识别
////	
////				u8g2_ClearBuffer(&u8g2);
////				u8g2_SetFont(&u8g2,u8g2_font_ncenB08_tf);
////				u8g2_DrawStr(&u8g2, 0,50,"MVMV  GOGO");
////				u8g2_SendBuffer(&u8g2);

//MVData[0]=0x00;
//	
//	while(MVData[0] != 0x36){
//		
//		if(MVData[0] == 0x34) {
//			
//			MVData[0]=0x00;
//			
//			
//			
////			runActionGroup(66, 1);
////			delay_ms(150);
////			runActionGroup(88, 1);
////			delay_ms(150);
//			
//			runActionGroup(77, 1);
//			runActionGroup(7, 1);//0x34分流进红球
//			delay_ms(340);
//			
//			
//			
//		}
//			
//		if(MVData[0] == 0x35) {
//			
//			MVData[0]=0x00;
//			
//			runActionGroup(8, 1);//0x35分流进黄球
//			
////			runActionGroup(66, 1);
////			delay_ms(150);
////			runActionGroup(88, 1);
////			delay_ms(150);
//			
//			runActionGroup(77, 1);
//			delay_ms(340);
//			
//			
//		}
//		
//	}
////	
}

	//*******************初始化完毕*******************//
//	char buffer[100];
//	while(1){
//		float dis1 = read_dis1();
//		float dis2 = read_dis2();
//		sprintf(buffer, "qian:%5.2f hou:%5.2f\r\n", dis1, dis2);
//		Usart_SendString(USART1, buffer);	
//	}
	/**
	*@brief 0x30-39是圆盘机通信数字，0x40-49是阶梯通信数字，0x50-59是立桩通信数字（括号内为发送方）
	
	*0x31确定红方（32），0x32确定蓝方（32），0x33圆盘机识别（32）,0x34分流进红（蓝）球，0x35分流进黄球 0x36圆盘机完成（k230）
	
	*0x40阶梯识别（32）,0x41确定要夹（k230），0x42夹紧爪子（32）,0x43松开爪子（32）
	
	*
	
	*/
	
	//*******************串口和屏幕调试区*******************//
	/*
	
	while(1){
		if(MVData[0] == 0x31 ){
			u8g2_ClearBuffer(&u8g2);
			u8g2_SetFont(&u8g2,u8g2_font_ncenB08_tf);
			u8g2_DrawStr(&u8g2, 0,50,"YES");
			u8g2_SendBuffer(&u8g2);
		}			
	}
	
//			u8g2_ClearBuffer(&u8g2);
//			u8g2_SetFont(&u8g2,u8g2_font_ncenB08_tf);
//			u8g2_DrawStr(&u8g2, 0,50,(char *)MVData);
//			u8g2_SendBuffer(&u8g2);
//	
//	
//	u8g2_ClearBuffer(&u8g2);
//	oled_print_float( 10,10,(float)( my_car.motor_1.encoder_count_all));
//	oled_print_float( 10,20,(float)( my_car.motor_2.encoder_count_all));
//	oled_print_float( 10,30,(float)( my_car.motor_3.encoder_count_all));
//	oled_print_float( 10,40,(float)( my_car.motor_4.encoder_count_all));
//	oled_print_float( 10,50,(float)( angle));
//	oled_print_float( 10,60,(float)( read_dis1()));
////	oled_print_float( 10,60,(float)( my_car.now_x));
////	oled_print_float( 60,60,(float)( my_car.now_y));
////	oled_print_float( 10,50,(float)( car.moter4.pwm_duty));
//	u8g2_SendBuffer(&u8g2);	
//	
	*/    
	
/*按下按键得知红蓝方*/
rb_t = 0;//按下按钮，1红，2蓝
	while(!rb_t)
	{
		if(read_key_red){
			rb_t = 1;
//			u8g2_ClearBuffer(&u8g2);
//			u8g2_SetFont(&u8g2,u8g2_font_ncenB08_tf);
//			u8g2_DrawStr(&u8g2, 0,50,"Red");
//			u8g2_SendBuffer(&u8g2);
		}
		if(read_key_blue){
			rb_t = 2;
//			u8g2_ClearBuffer(&u8g2);
//			u8g2_SetFont(&u8g2,u8g2_font_ncenB08_tf);
//			u8g2_DrawStr(&u8g2, 0,50,"Blue");
//			u8g2_SendBuffer(&u8g2);
		}
	}
	
	Usart_SendByte(UART5, (0x30+rb_t));//发送字符1或2给mv确定红蓝方
	
	
	runActionGroup(10, 1);		//确认红蓝方后所有机械臂复位
	
	
	//*******************圆盘机跑图*******************//
	
	if(rb_t==1)//hong
{

/*红方圆盘机*/
	if(1)		
	{	
			
//			go_to_xy(-55,100,397,100,100,100);			// 去往圆盘机
		
			go_to_xy(-53,100,418,100,100,100);			// 换方案				415.5
		
//		    NoWay = 0;
			my_car.target_yaw = 270;                     // 转角度
			while(!(abs(10*(my_car.yaw-270))<5));        //等待转90完成
//			NoWay = 0.005;
			delay_ms(400);//给他一点时间去调整
		
			runActionGroup(4, 1);//机械臂抬起，该动作组运行时间为1秒
			delay_ms(1300);
		
		//前面的漫反射找到白线
				if(1)
			{
				int i = 0;
				i = read_MANFAN1;
				y_set_speed_flag = 1;
				my_car.v_y = 20;
				while(i == read_MANFAN1);
				my_car.v_y = 0;
				y_set_speed_flag = 0;
				go_to_xy(0,0,-1,10,10,10);
			}
			
			if(0)
			{
				/*
//				x_set_speed_flag = 1;
//				my_car.v_x = -5; //左移
//				
//				int posi1 = read_lhuidu;
//				int posi2,posi3 ;
//				while(1)
//				{
//					
//					posi1 =  read_lhuidu;    
//					posi2 =  read_mhuidu;
//					posi3 =  read_rhuidu;
//					
//					u8g2_ClearBuffer(&u8g2);
//					oled_print_float(10,20,posi1); //行，列
//					oled_print_float(10,40,posi2);
//					oled_print_float(10,60,posi3);
//					u8g2_SendBuffer(&u8g2);
//					 if(posi1 == 1 && posi2 == 0 && posi3 == 1)	//只有中间灰度能扫到白线
//						{
//							my_car.v_x = 0;
//							x_set_speed_flag = 0;
//							break;
//						}
//						
//				}
//				delay_ms(500);
//				go_to_xy(1.5,100,0,120,90,100);			// 调整位置
//			

					*/
//				int j=0;
//				j = read_MANFAN1;
//				x_set_speed_flag = 1;
//				my_car.v_x = -20;                              //2025.10.8暂时换方案
//				while(j == read_MANFAN1);
//				my_car.v_x = 0;
//				x_set_speed_flag = 0;
//			


//				Usart_SendByte(USART1, '6');//发送字符6给底盘MV识别白线
//				x_set_speed_flag = 1;
//				my_car.v_x = -3;
//				while(MVData2[0] != '7');
//				my_car.v_x = 0;
//				x_set_speed_flag = 0;				
//				
				/////////
//				go_to_xy(33,20,0,0,20,20);        //漫反射找到白线边缘后，往右边走到中间
//												  //应该是厘米数，go_to_xy 函数单位是cm（不对，是一定比例）
//				
////				go_to_xy(0,20,-0.56,20,20,20);//临时往后走一格
//				go_to_xy(0, 20, 5.9, 20, 20,20);//临时往后走一格
												  
//				if(1)//右移后会有前后偏差，再前后调整
//			{
//				int i = 0;
//				i = read_MANFAN1;
//				y_set_speed_flag = 1;
//				my_car.v_y = 5;
//				while(i == read_MANFAN1);
//				my_car.v_y = 0;
//				y_set_speed_flag = 0;
//			}

			}		
	
	} 
	
	//*******************圆盘机拍球*******************//
	
	
		runActionGroup(78, 1);//机械臂拍球动作
		delay_ms(1400);
		Usart_SendByte(UART5, 0x33);//告诉mv开始识别

	MVData[0]=0x00;
	uint8_t first_flag = 0;
	
	while(MVData[0] != 0x36){
		
		if(MVData[0] == 0x34) {//红
			if(!first_flag){
				
				first_flag++;
				MVData[0]=0x00;
			}
			else {
				
				runActionGroup(77, 1);
				delay_ms(350);
				MVData[0]=0x00;
			}
			
		}
			
		if(MVData[0] == 0x35) {//黄
			if(!first_flag){
				
				first_flag++;
				MVData[0]=0x00;
			}
			else {
				
				runActionGroup(99, 1);
				delay_ms(350);
				MVData[0]=0x00;
			}
			
		}
	}
	
	
	while(MVData[0] != 0x36);    //mv端任务完成

				
/*红方阶梯*/
	

	runActionGroup(4, 1); //机械臂抬起防止误拍
	delay_ms(2000);
	
	//避障方案一：（固定障碍物）
//	go_to_xy(-74,100,-35,100,100,100); //避开障碍物
//	runActionGroup(10, 1); //机械臂收起防止撞到阶梯
//	go_to_xy(-37,100,0,100,100,100); 
//	go_to_xy(-100,50,-100,100,100,100);   

	//避障方案二：（可调障碍物）
	go_to_xy(0,50,-35,50,50,50); //避开障碍物
	runActionGroup(20, 1); //机械臂收起防止撞到阶梯
	go_to_xy(-110,60,0,60,60,60); 
	go_to_xy(-62,50,-115,100,100,100);   
	

//	NoWay = 0;
	my_car.target_yaw = 90;			
	while(!(abs(10*(my_car.yaw-90))<5));
//	NoWay = 0.005;
	
   
		//*******************阶梯任务*******************//
		
		jieti2223();


    /*2025.8.30 这里师兄用的是测距模块 暂时改成用激光模块 看看效果如何*/         //2025.10.10待写

//方案一
//	while(1)                                    //x距离纠正
//	{
//		float dis = read_dis1();                  
////		if(abs((int)dis)<95)break;
//		if(abs((int)dis)<95)break;
//		else{
//			go_to_xy(1,90,0,0,60,60);
//		}
//	}

//while(1){
//		float dis = read_dis1();
////		if(dis < 110 || dis > 160){                                     //若阶梯边沿有距离很近的障碍物，则启用该方案，否则95
////		if(dis < 95){
//		if(dis > 200){													//已经好了，大于20厘米即可
//			break;
//		}else{
//			go_to_xy(-2,40,0,100,20,100);
////			char buffer[100];
////		    sprintf(buffer, "qian:%5.2f\r\n", dis);
////		    Usart_SendString(UART5, buffer);
//		}
//	}

//	{
//		int i=0;
//		i = read_MANFAN2;
//		x_set_speed_flag = 1;
//		my_car.v_x = 20;
//		while(i == read_MANFAN2);
//		my_car.v_x = 0;
//		x_set_speed_flag = 0;
//	}
	
	
	
                               /*红方仓库倒圆盘机的球*/
	
//		NoWay = 0;
		my_car.target_yaw = 90;		                     //车不正，微调一下	
		while(!(abs(10*(my_car.yaw-90))<5));
//		NoWay = 0.005;
	
	
	if(1)        
	{
		go_to_xy(0, 100, -130, 100, 100, 100);           //阶梯处后退往仓库倒球 150要调
		go_to_xy(-55,100,0,90,100,100);                  //仓库处往中间移
		
		while(1)                                      
	{
		float dis = read_dis2();						//测距后退紧贴仓库（本来是35 改成37试一下）		34（今天早上都在用）
		if(dis > 600) continue;							//停不下来的数据是2490，简单滤波算法跳出本次循环（一般是2-5次）
		if(abs((int)dis-33)<2)break;
		else{
			go_to_xy(0,90,-(dis-33)/15,30,30,30);       //偶尔退的太快，速度改小
		}
	}  
	
	
	while(!read_MANFAN3)						//测得到是0，测不到是1
	{
		go_to_xy(-1,30,0,90,30,60);
	}
	
	go_to_xy(0,20,1,20,20,20);					//往前走一格就不会卡住
	go_to_xy(3.9,20,0,20,20,20);					//红方仓库有凸起，单独要加往右距离
	delay_ms(1000);
	
	runActionGroup(51, 1); 	// 倒球
	delay_ms(2000);
		
	for(int i = 0;i<3;i++)
				{
					go_to_xy(0,90,2,200,30,150);
					go_to_xy(0,90,-2,200,30,150);
				
				}
	delay_ms(2000);
				
	}
	runActionGroup(50, 1); 	// 收倒球槽
	delay_ms(2000);
	runActionGroup(209,1);//立桩机械臂抬起
	delay_ms(2500);
	
	my_car.target_yaw = 90;		                     //车不正，微调一下	
	while(!(abs(10*(my_car.yaw-90))<5));
	
/*红方立桩*/
	
		lizhuang2221();
	
	
/*红方仓库倒立桩的球和阶梯的方块*/
	
//	go_to_xy(-5, 20, 0, 20, 20, 20);//转弯圈还会往右挪，要往左一点，不然会碰到球
//	runActionGroup(210,1);//立桩机械臂抬起，防止触球
//	delay_ms(1500);

//	NoWay = 0;
//	my_car.target_yaw = 90;		                     //车不正，微调一下	
//	while(!(abs ( 10 * ( my_car.yaw - 90)) < 5 ));
//	NoWay = 0.005;
	
	if(1)
	{
					go_to_xy(0, 90, -20, 90, 100, 100);           //阶梯处后退往仓库倒球 150要调
					go_to_xy(-10,90,0,90,30,60);                  //仓库处往中间移
		
		runActionGroup(10, 1);//全部收起
	    delay_ms(3000);			

			  my_car.target_yaw = 90;                     // 转角度
      while(!(abs(10*(my_car.yaw-90))<5));   
				while(1)                                    
				{
					float dis = read_dis2();                  
					if(dis > 600) continue;							//停不下来的数据是2490，简单滤波算法跳出本次循环（一般是2-5次）
					if(abs((int)dis-33)<5) break;//44（本来是35 改成37试一下）
					else{
						go_to_xy(0,90,-(dis-33)/15,10,10,10);
					}
				}
				
				while(!read_MANFAN3)						//测得到是0，测不到是1
				{
					go_to_xy(-1,30,0,90,30,60);
				}

				go_to_xy(0,20,1,20,20,20);					//往前走一格就不会卡住
				go_to_xy(3.9,20,0,20,20,20);
				delay_ms(1000);

				runActionGroup(51, 1); 	// 倒球
				delay_ms(2000);
				
				for(int i = 0;i<3;i++)	//把车里的球抖出来
				{
					go_to_xy(0,90,2,200,30,150);
					go_to_xy(0,90,-2,200,30,150);
				}
				delay_ms(2000);
				runActionGroup(50, 1);//收倒球槽
	
	
	}

	
	//*******************在仓库倒积木块*******************//
	
	
	if(1)
	{		

	  go_to_xy(0,90,20,90,60,60);//靠近立桩的方向走前一段（因为是前面倒积木）
		
//	  NoWay = 0;
	  my_car.target_yaw = 270;                     // 转角度
      while(!(abs(10*(my_car.yaw-270))<5));   
//	  NoWay = 0.005;
		
		go_to_xy(-4.5,90,0,90,60,60);				//往左走
		
		while(1)                                    
		{
			float dis = read_dis1();                  
//			if( abs((int)dis-35) < 5 ) break;//44
//			else{
//				go_to_xy(0,90,(dis-35)/15,90,20,70);
//			}
			if( abs((int)dis-69) < 5 ) break;//44			84				70（10.17中午在用）
			else{
//				go_to_xy(0,90,(dis-70)/15,90,20,70);
				go_to_xy(0,30,(dis-69)/20,30,30,30);
			}
			
		}
		runActionGroup(220, 1);//释放积木块到仓库
		delay_ms(5000);
	
	}
	
	
/*红方回家*/
	
	
//	go_to_xy(0,90,-20,90,60,60);//靠近立桩的方向走前一段
	
//	NoWay = 0;
	my_car.target_yaw = 0;
	while(!(abs(10*(my_car.yaw-0))<5));
	delay_ms(500);
//	NoWay = 0.005;

	go_to_xy(0,60,-226,100,60,100);
	go_to_xy(80,100,0,100,100,100);
	//GoBack();					// 返回起点

	while(1);					// 结束	
}





//////////////////上面红方代码，下面为蓝方/////////////////////////





	
//*******************圆盘机跑图*******************//
	
	if(rb_t==2)//lan
{

/*蓝方圆盘机*/
	if(1)		
	{	
			
			go_to_xy(53,100,423,100,100,100);			// 换方案        原来是50，x不够大会容易撞到立桩
		
//		    NoWay = 0;
			my_car.target_yaw = 90;                     // 转角度
			while(!(abs(10*(my_car.yaw-90))<5));        //等待转90完成
//			NoWay = 0.005;
			delay_ms(400);//给他一点时间去调整
		
			runActionGroup(4, 1);//机械臂抬起，该动作组运行时间为1秒
			delay_ms(1300);
		
		//前面的漫反射找到白线
				if(1)
			{
				int i = 0;
				i = read_MANFAN1;
				y_set_speed_flag = 1;
				my_car.v_y = 20;
				while(i == read_MANFAN1);
				my_car.v_y = 0;
				y_set_speed_flag = 0;
				go_to_xy(0,0,-1,10,10,10);
			}
	
	} 
	
	
	
	//*******************圆盘机拍球*******************//
	
	
		runActionGroup(78, 1);//机械臂拍球动作，该动作组运行时间为1秒
		delay_ms(1400);
		Usart_SendByte(UART5, 0x33);//告诉mv开始识别

	MVData[0]=0x00;
	uint8_t first_flag = 0;
	
	while(MVData[0] != 0x36){
		
		if(MVData[0] == 0x34) {//蓝
			if(!first_flag){
				
				first_flag++;
				MVData[0]=0x00;
			}
			else {
				
				runActionGroup(77, 1);
				delay_ms(350);
				MVData[0]=0x00;
			}
			
		}
			
		if(MVData[0] == 0x35) {//黄
			if(!first_flag){
				
				first_flag++;
				MVData[0]=0x00;
			}
			else {
				
				runActionGroup(99, 1);
				delay_ms(350);
				MVData[0]=0x00;
			}
			
		}
	}
	
	
	while(MVData[0] != 0x36);    //mv端任务完成


				
/*蓝方阶梯*/
	

	runActionGroup(4, 1); //机械臂抬起防止误拍
	delay_ms(2000);
	
	
	//避障方案一:（固定障碍物）
//	go_to_xy(74,100,-35,100,100,100); //避开障碍物
//	runActionGroup(10, 1); //机械臂收起防止撞到阶梯
//	go_to_xy(37,100,0,100,100,100); 
//	go_to_xy(120,50,-115,100,50,100); 
	
	//避障方案二:（可调障碍物）
	go_to_xy(0,50,-35,50,50,50); //避开障碍物
	runActionGroup(20, 1); //机械臂收起防止撞到阶梯
	go_to_xy(110,60,0,60,60,60);
	go_to_xy(110,50,-115,100,50,100);
	

//	NoWay = 0;
	my_car.target_yaw = 270;			
	while(!(abs(10*(my_car.yaw-270))<5));
//	NoWay = 0.005;
	
//	runActionGroup(109, 1);//高阶梯识别（已张开）    2700ms
//	delay_ms(3000);			//防止抬起来顶到阶梯，需要提前举起来
	
   
		//*******************阶梯任务*******************//


		jieti2223();


		//激光离开
//	{
//		int i=0;
//		i = read_MANFAN2;
//		x_set_speed_flag = 1;
//		my_car.v_x = 20;
//		while(i == read_MANFAN2);
//		my_car.v_x = 0;
//		x_set_speed_flag = 0;
//	}

/*
		while(1){
		float dis = read_dis1();
//		if(dis < 105 || dis > 160){                                     //若阶梯边沿有距离很近的障碍物，则启用该方案，否则95
//		if(dis < 95){
		if(dis > 200){
			break;
		}else{
			go_to_xy(2,40,0,100,20,100);//往右离开
//			char buffer[100];
//		    sprintf(buffer, "qian:%5.2f\r\n", dis);
//		    Usart_SendString(UART5, buffer);
		}
	}
	*/
                               /*蓝方仓库倒圆盘机的球*/
	
//		NoWay = 0;
		my_car.target_yaw = 270;		                     //车不正，微调一下	
		while(!(abs(10*(my_car.yaw-270))<5));
//		NoWay = 0.005;
	
	
	if(1)        
	{
		go_to_xy(0, 100, -130, 100, 100, 100);           //阶梯处后退往仓库倒球 150要调
		go_to_xy(-55,100,0,90,100,100);                  //仓库处往中间移
		
		while(1)                                      
	{
		float dis = read_dis2();						//测距后退紧贴仓库（本来是35 改成37试一下）
		if(dis > 600) continue;							//停不下来的数据是2490，简单滤波算法跳出本次循环（一般是2-5次）
		if(abs((int)dis-33)<2)break;
		else{
			go_to_xy(0,90,-(dis-33)/15,30,30,30);      
		}
	}  
	
	while(!read_MANFAN3)						//测得到是0，测不到是1
	{
		go_to_xy(-1,30,0,90,30,60);
	}
	
	go_to_xy(0,20,1,20,20,20);					//往前走一格就不会卡住
	go_to_xy(3.8,20,0,20,20,20);
	delay_ms(1000);
	
	runActionGroup(51, 1); 	// 倒球
	delay_ms(2000);
		
	for(int i = 0;i<3;i++)
				{
					go_to_xy(0,90,2,200,30,150);
					go_to_xy(0,90,-2,200,30,150);
				
				}
	delay_ms(2000);
				
	}
	
	runActionGroup(50, 1); 	// 收倒球槽
	delay_ms(2000);
	runActionGroup(209,1);//立桩机械臂抬起
	delay_ms(2500);
	
	my_car.target_yaw = 270;		                     //车不正，微调一下	
	while(!(abs(10*(my_car.yaw-270))<5));
	
/*蓝方立桩*/
	
	lizhuang2221();
	
	
/*蓝方仓库倒立桩的球和阶梯的方块*/
	
//	go_to_xy(-5, 20, 0, 20, 20, 20);//转弯圈还会往右挪，要往左一点，不然会碰到球
//	runActionGroup(210,1);//立桩机械臂抬起，防止触球
//	delay_ms(1500);

//	NoWay = 0;
//	my_car.target_yaw = 267;		                     //车不正，微调一下	
//	while(!(abs ( 10 * ( my_car.yaw - 267)) < 5 ));
//	NoWay = 0.005;
	
	if(1)
	{
					go_to_xy(0, 90, -20, 90, 100, 100);           //阶梯处后退往仓库倒球 150要调
					go_to_xy(-10,90,0,90,30,60);                  //仓库处往中间移
		
		runActionGroup(10, 1);//全部收起
	    delay_ms(3000);			

//			  my_car.target_yaw = 267;                     // 转角度
//      while(!(abs(10*(my_car.yaw-267))<5));   
				while(1)                                    
				{
					float dis = read_dis2();                  
					if(dis > 600) continue;							//停不下来的数据是2490，简单滤波算法跳出本次循环（一般是2-5次）
					if(abs((int)dis-33)<5) break;//44（本来是35 改成37试一下）
					else{
						go_to_xy(0,90,-(dis-33)/15,10,10,10);
					}
				}
				
				while(!read_MANFAN3)						//测得到是0，测不到是1
				{
					go_to_xy(-1,30,0,90,30,60);
				}

				go_to_xy(0,20,1,20,20,20);					//往前走一格就不会卡住
				go_to_xy(3.8,20,0,20,20,20);
				delay_ms(1000);

				runActionGroup(51, 1); 	// 倒球
				delay_ms(2000);
				
				for(int i = 0;i<3;i++)	//把车里的球抖出来
				{
					go_to_xy(0,90,2,200,30,150);
					go_to_xy(0,90,-2,200,30,150);
				}
				delay_ms(2000);
				runActionGroup(50, 1);//收倒球槽
	
	}

	
	//*******************在仓库倒积木块*******************//
	
	
	if(1)
	{		

	  go_to_xy(0,90,20,90,60,60);//靠近立桩的方向走前一段（因为是前面倒积木）
		
//	  NoWay = 0;
	  my_car.target_yaw = 90;                     // 转角度
      while(!(abs(10*(my_car.yaw-90))<5));   
//	  NoWay = 0.005;
		
		go_to_xy(-4.5,90,0,90,60,60);//往左走
		
		while(1)                                    
		{
			float dis = read_dis1();                  
			if( abs((int)dis-69) < 5 ) break;//44            35				84				70（10.17中午在用）
			else{
//				go_to_xy(0,90,(dis-70)/15,90,20,70);
				go_to_xy(0,30,(dis-69)/20,30,30,30);
			}
			
		}
		runActionGroup(220, 1);//释放积木块到仓库
		delay_ms(5000);
	
	}
	
	
/*蓝方回家*/
	
//	NoWay = 0;
	my_car.target_yaw = 0;
	while(!(abs(10*(my_car.yaw-0))<5));
	delay_ms(500);
//	NoWay = 0.005;

	go_to_xy(0,60,-198,100,60,100);							//分段走比较稳
	go_to_xy(-79,100,0,100,100,100);
	//GoBack();					// 返回起点

	while(1);					// 结束	
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////下面是一些函数///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

if(0)
{
if(1)
{
go_to_xy(-100,90,0,90,60,60);
go_to_xy(-15,90,-110,90,60,60);
go_to_xy(80,90,0,90,60,60);
cangkufangqiu();
go_to_xy(0,90,10,90,60,60);
lizhuang();
go_to_xy(0,90,-10,90,60,60);
cangkufangqiu();
my_car.target_yaw = 0;
while(!(abs(10*(my_car.yaw))<5));
go_to_xy(-60,90,-300,90,60,60);
while(1);
if(1)
{
  int i=0;
	i = read_MANFAN2;
	x_set_speed_flag = 1;
	if(i)my_car.v_x = 5;
	else my_car.v_x = -5;
	while(i == read_MANFAN2);
	my_car.v_x = 0;
	x_set_speed_flag = 0;
}
while(1)
{
	float dis = read_dis1();
	if(abs((int)dis-30)<2)break;
	else{
		go_to_xy(0,90,(dis-30)/15,90,20,60);
	}
}

while(1);
}


///////////////////////////////





go_to_xy(55,100,395,130,20,100);
//while(1)
//{
//	if((abs(10*(my_car.now_x-60))<5)&&(abs(10*(my_car.now_y-385))<5))break;
//	else{
//		go_to_xy(60-my_car.now_x,90,385-my_car.now_y,90);
//	}
//}
my_car.target_yaw = 90;
while(!(abs(10*(my_car.yaw-90))<5));
while(1)
{
	float dis = read_dis1();
	if(abs((int)dis-30)<2)break;
	else{
		go_to_xy(0,90,(dis-30)/15,90,30,60);
	}
}
if(1)
{
  int i=0;
	i = read_MANFAN1;
	x_set_speed_flag = 1;
	if(i)my_car.v_x = -5;
	else my_car.v_x = 5;
	while(i == read_MANFAN1);
	my_car.v_x = 0;
	x_set_speed_flag = 0;
}

runActionGroup(1, 1); //机械臂复位
delay_ms(500);
Usart_SendByte( UART5, 0x31);
MVData[0]=0;
while(MVData[0] != 0x33);
runActionGroup(2, 1); //机械臂复位
while(1)
{
	float dis = read_dis2();
	if(dis<250)break;
	else{
		go_to_xy(0,90,-1,90,30,60);
	}
}
go_to_xy(20,90,0,90,30,60);
go_to_xy(120,100,-80,100,60,60);
my_car.target_yaw = 270;
while(!(abs(10*(my_car.yaw-270))<5));
//Usart_SendByte( UART5, 0x00);
//TIM_SetCompare1(TIM8,1000);
while(1)
{	
	u8g2_ClearBuffer(&u8g2);
	oled_print_float( 10,10,(float)( my_car.motor_1.encoder_count_all));
	oled_print_float( 10,20,(float)( my_car.motor_2.encoder_count_all));
	oled_print_float( 10,30,(float)( my_car.motor_3.encoder_count_all));
	oled_print_float( 10,40,(float)( my_car.motor_4.encoder_count_all));
	oled_print_float( 10,50,(float)( angle));
	oled_print_float( 10,60,(float)( read_dis1()));
//	oled_print_float( 10,60,(float)( my_car.now_x));
//	oled_print_float( 60,60,(float)( my_car.now_y));
//	oled_print_float( 10,50,(float)( car.moter4.pwm_duty));
	u8g2_SendBuffer(&u8g2);	

}
}
}


	









char buffer[50];
//纵向速度控制线程入口函数
volatile float speed_dir_x = 1.0f;
volatile float speed_dir_y = 1.0f;
volatile uint8_t x_speed_plan_flag = 0;//1速度规划完，0执行完，或者静止状态
volatile uint8_t y_speed_plan_flag = 0;
extern volatile float ti ;
void go_to_xy(float x_distance,float x_speed_plan,float y_distance,float y_speed_plan,float a_x,float a_y)
{//参数：x方向移动距离，x方向最大速度，y方向移动距离，y方向最大速度，x方向最大加速度，y方向最大加速度
	if(x_distance > 0.0f)
	{
		x_distance = x_distance;
		speed_dir_x = 1.0f;
	}
	else
	{
		x_distance = -1.0f*x_distance;
		speed_dir_x = -1.0f;
	}
	//计算一次路程所需的速度规划(ti >= tp_y.t)
	calcTrapezoidalProfile(x_distance/1.0f,0.0f,x_speed_plan,0.0f,a_x,a_x,&tp_x);

	if(y_distance > 0.0f)
	{
		y_distance = y_distance;
		speed_dir_y = 1.0f;
	}
	else
	{
		y_distance = -1.0f*y_distance;
		speed_dir_y = -1.0f;
	}
	//计算一次路程所需的速度规划(ti >= tp_y.t)
	calcTrapezoidalProfile(y_distance/1.0f,0.0f,y_speed_plan,0.0f,a_y,a_y,&tp_y);
	
	ti = 0;
	x_speed_plan_flag = 1;
	y_speed_plan_flag = 1;
//	while(x_speed_plan_flag||y_speed_plan_flag);//等待pid执行完
	while(x_speed_plan_flag||y_speed_plan_flag);
		//等待pid执行完
	
}


void lizhuang222()
{ 
	/////////单独调试用
	//runActionGroup(0,1);
	//delay_ms(5000);
	
	/////////////////////////
	go_to_xy(0,90,30,90,60,60);//靠近立桩的方向走前一段
	//先抬高一点机械臂
	runActionGroup(44,1);
	delay_ms(2500);
	
	int i=0;
	i = read_MANFAN2;
	x_set_speed_flag = 1;
	if(i)my_car.v_x = 2;
	else my_car.v_x = -2;
	while(i == read_MANFAN2);
	my_car.v_x = 0;
	x_set_speed_flag = 0;
	y_set_speed_flag = 1;
	
	//对立桩柱子测距
	while(1)
	{
		float dis = read_dis1();
		if(abs((int)dis-162)<5)break;
		else{
			go_to_xy(0,90,(dis-162)/15,90,30,60);    ////172
		}
	}
	y_set_speed_flag = 0;
	
	/***************************2025.8.17
	
	//机械臂准备扫描校准
	runActionGroup(40,1);
	delay_ms(2500);
	Usart_SendByte( UART5, 0x39);
	/////////////////////////////////需不需要校准？////////////////////////////////
	
	while(1)
	{
		
		if(MVData[0])//7qian  8hou  zuo9  you0
		{
			if(MVData[0] == 0x37){go_to_xy(0,10,0.1,10,60,60);MVData[0]=0;Usart_SendByte( UART5, 0x31);}
			else if(MVData[0] == 0x38){go_to_xy(0,10,-0.1,10,60,60);MVData[0]=0;Usart_SendByte( UART5, 0x31);}
			else if(MVData[0] == 0x39){go_to_xy(-0.1,10,0,10,60,60);MVData[0]=0;Usart_SendByte( UART5, 0x31);}
			else if(MVData[0] == 0x30){go_to_xy(0.1,10,0,10,60,60);MVData[0]=0;Usart_SendByte( UART5, 0x31);}
			else if(MVData[0] == 0x32){MVData[0]=0;Usart_SendByte( UART5, 0x31);}
			else if(MVData[0] == 0x33){MVData[0]=0;Usart_SendByte( UART5, 0x32);break;}//校准完成
		}
	}
				u8g2_ClearBuffer(&u8g2);
				u8g2_SetFont(&u8g2,u8g2_font_ncenB08_tf);
				u8g2_DrawStr(&u8g2, 0,50,"JIAOZHUN  OK");
				u8g2_SendBuffer(&u8g2);
	//////////////////////////////加了校准，下面的好像还没调好///////////////////////////////////////////////////
   delay_ms(1000);
	//准备夹球
	runActionGroup(26, 1);	
	delay_ms(3500);
	//发送夹球指令
	Usart_SendByte( UART5, 0x37);
	delay_ms(1000);
	//把球放到小仓库上头
	runActionGroup(27, 1);	
	delay_ms(5000);
	//发送放球指令
	Usart_SendByte( UART5, 0x38);
	delay_ms(2000);
	//夹完球复位
	runActionGroup(28, 1);	
	delay_ms(4000);
	//抬高机械臂
	runActionGroup(25,1);
	delay_ms(2500);
	//需不需要再调整一下y位置？
	//		while(1)
	//	{
	//		float dis = read_dis1();
	//		if(abs((int)dis-172)<5)break;
	//		else{
	//			go_to_xy(0,90,(dis-172)/15,90,30,60);    ////195
	//		}
	//	}
	//架好准备转圈识别和拍球
	 runActionGroup(23, 1);	
	 delay_ms(3000);
	//发送开启识别和拍球指令
	Usart_SendByte( UART5, 0x39);
	
	**************************************/
	
	//转圈拍球
	write_cri();
	delay_ms(2000);
}

	
void lizhuang2221()
{ 
	
	go_to_xy(0,90,30,90,60,60);//靠近立桩的方向走前一段
//	go_to_xy(0,90,40,90,60,60);
	
	int i=0;
	i = read_MANFAN2;
	x_set_speed_flag = 1;
	if(i)my_car.v_x = 2;
	else my_car.v_x = -2;
	while(i == read_MANFAN2);
	my_car.v_x = 0;
	x_set_speed_flag = 0;
	y_set_speed_flag = 1;
	
	runActionGroup(7, 1);		//分流板摆到蓝边
	delay_ms(500);
	
	//对立桩柱子测距
	while(1)
	{
		float dis = read_dis1();
		if(abs((int)dis-172)<5)break;			//162太近了
		else{    
			go_to_xy(0,90,(dis-172)/15,30,30,30);    
//			char buffer[100];
//			sprintf(buffer, "qian:%5.2f\r\n", dis);
//			Usart_SendString(UART5, buffer);
		}
	}
	y_set_speed_flag = 0;
	

/*不需要校准2025.9.26
//	while(1)
//	{
//		
//		if(MVData[0])//(默认MV发给32)0x51qian  0x52hou  0x53zuo  0x54you  0x55继续校准(32发给MV)   0x56结束识别
					 //				 0x57夹球(32给MV)	0x58放球(32给MV)		0x59开始拍球(32给MV)		0x60结束立桩
	
//		{
//			if(MVData[0] == 0x51){go_to_xy(0,10,0.1,10,60,60);MVData[0]=0x00;Usart_SendByte( UART5, 0x55);}
//			else if(MVData[0] == 0x52){go_to_xy(0,10,-0.1,10,60,60);MVData[0]=0x00;Usart_SendByte( UART5, 0x55);}
//			else if(MVData[0] == 0x53){go_to_xy(-0.1,10,0,10,60,60);MVData[0]=0x00;Usart_SendByte( UART5, 0x55);}
//			else if(MVData[0] == 0x54){go_to_xy(0.1,10,0,10,60,60);MVData[0]=0x00;Usart_SendByte( UART5, 0x55);}
////			else if(MVData[0] == 0x32){MVData[0]=0;Usart_SendByte( UART5, 0x31);}
////			else if(MVData[0] == 0x56){MVData[0]=0;Usart_SendByte( UART5, 0x32);break;}//校准完成
//			else if(MVData[0] == 0x56){MVData[0]=0x00;break;}//校准完成
//		}
//	}
//				u8g2_ClearBuffer(&u8g2);
//				u8g2_SetFont(&u8g2,u8g2_font_ncenB08_tf);
//				u8g2_DrawStr(&u8g2, 0,50,"JIAOZHUN  OK");
//				u8g2_SendBuffer(&u8g2);
*/


   
	
	go_to_xy(-4,60,0,60,60,60);
	go_to_xy(0,60,-3.5,60,60,60);
	
	runActionGroup(55, 1);
	delay_ms(1000);
	
	//运行夹球动作组
	runActionGroup(203, 1);	
	delay_ms(3000);
	//发送夹球指令
//	Usart_SendByte( UART5, 0x57);
	runActionGroup(66, 1);
	delay_ms(1000);
	
	go_to_xy(0,20,-2,20,20,20);//往后一点，防止碰到球
	
	//运行放球动作组
	runActionGroup(205, 1);	
	delay_ms(4800);
	
	//发送放球指令
//	Usart_SendByte( UART5, 0x58);
	
	runActionGroup(88, 1);
	delay_ms(1000);
	
	runActionGroup(66, 1);	//爪子收起
	delay_ms(200);
	
	
	/*不做绕圈
	
	
	//一口气转圈法：
	//	圆心不对，调整一下
    go_to_xy(5.15,60,0,60,60,60);
	go_to_xy(0,60,3.15,60,60,60);					
	delay_ms(1000);

	runActionGroup(207, 1);	//立桩拍球
	delay_ms(2400);
	
//	//测距定点校准法：
//	go_to_xy(0,20,2,20,20,20);				//夹白球已经是中间位置，把前后抵消即可
	
	//发送开启识别和拍球指令
	Usart_SendByte( UART5, 0x61);
	//转圈拍球
	write_cri();
	delay_ms(2000);	
	Usart_SendByte( UART5, 0x63);//告诉MV立桩结束
	*/
	
}
	
	
	
void WareHouse(void)
{
	//纠正一下角度
//	my_car.target_yaw = 270;
//	while(!(abs(10*(my_car.yaw-270))<5));

	go_to_xy(0, 90, -150, 100, 100, 100);           //阶梯处后退往仓库倒球 150要调
	go_to_xy(-60,100,0,90,100,100);                  //仓库处往中间移


	while(1)                                      //测距后退紧贴仓库
	{
		float dis = read_dis2();
		if(abs((int)dis-37)<2)break;
		else{
			go_to_xy(0,90,-(dis-37)/15,90,30,60);
		}
	}  
	
	while(!read_MANFAN3)						//测得到是0，测不到是1
	{
		go_to_xy(-1,30,0,90,30,60);
	}
	go_to_xy(2,20,0,90,30,60);
	delay_ms(1000);

//	runActionGroup(51, 1); 	// 放球
//	delay_ms(2000);
		for(int i = 0;i<3;i++)
	{
		go_to_xy(0,90,2,200,30,150);
		go_to_xy(0,90,-2,200,30,150);
	
	}
	delay_ms(2000);

//	runActionGroup(201, 1); 	// 复位
//	delay_ms(4000);
				
/*if(1){
				int ig=0;
		ig = read_MANFAN3;
		while(1)
		{
			if(ig == read_MANFAN3)
			{
				if(ig)go_to_xy(0.5,10,0,90,30,60);
				else go_to_xy(-0.5,10,0,90,30,60);
			}
			else 
			{
			go_to_xy(11.5,10,0,90,30,60);
				break;
			}
	}

}*/
}
void LiZhuang(void)
{
	
	runActionGroup(225, 1); 			// 抬高机械臂
	delay_ms(2000);
	go_to_xy(13.5, 90, 0, 0, 100, 100);
	go_to_xy(0, 0, 34, 90, 100, 100);
	/*	runActionGroup(223, 1); 
	delay_ms(1000);
	Usart_SendByte( UART5, 0x37);
	delay_ms(1000);
	runActionGroup(222, 1); 
	delay_ms(1500);
	Usart_SendByte( UART5, 0x38);
	delay_ms(1000);*/
	delay_ms(1000);
	runActionGroup(221, 1); 		 // 机械臂伸出
	delay_ms(3000);
	
	runActionGroup(220, 1);				// 机械臂MV对准
	delay_ms(2000);
	Usart_SendByte(UART5, 0x31);		// MV校准
	//while(1);

	MVData[0]=0;
	while(1)
	{
		
		if(MVData[0])//7qian  8hou  zuo9  you0
		{
			if(MVData[0] == 0x37){go_to_xy(0,10,0.3,10,60,60);MVData[0]=0;Usart_SendByte( UART5, 0x31);}
			else if(MVData[0] == 0x38){go_to_xy(0,10,-0.3,10,60,60);MVData[0]=0;Usart_SendByte( UART5, 0x31);}
			else if(MVData[0] == 0x39){go_to_xy(-0.3,10,0,10,60,60);MVData[0]=0;Usart_SendByte( UART5, 0x31);}
			else if(MVData[0] == 0x30){go_to_xy(0.3,10,0,10,60,60);MVData[0]=0;Usart_SendByte( UART5, 0x31);}
			else if(MVData[0] == 0x32){MVData[0]=0;Usart_SendByte( UART5, 0x31);}
			else if(MVData[0] == 0x33){MVData[0]=0;Usart_SendByte( UART5, 0x32);break;}//校准完成
		}
	}
	
	u8g2_ClearBuffer(&u8g2);
	u8g2_SetFont(&u8g2, u8g2_font_inb24_mf);
	u8g2_DrawStr(&u8g2, 30,50,"OKOKOKOK");
	u8g2_SendBuffer(&u8g2);
	
	runActionGroup(219, 1);		 // 立桩夹中间球
	delay_ms(3000);
	Usart_SendByte(UART5, 0x37);		// 发送MV夹
	
	delay_ms(1000);
	runActionGroup(218, 1);			// 机械臂放球
	delay_ms(3000);
	Usart_SendByte(UART5, 0x38);		// 发送MV放
	delay_ms(1000);
	runActionGroup(221, 1);				// 机械臂伸出准备拍
	delay_ms(3000);
	Usart_SendByte(UART5, 0x31);		// 告诉MV开始识别
	
	write_cri();					// 转圈
	delay_ms(1000);
	go_to_xy(0, 0, -35, 90, 100, 100);

	go_to_xy(-10, 90, 0, 0, 100, 100);
	while(read_MANFAN3)
	{
		go_to_xy(0,0,-1,90,30,60);
		
	}
	go_to_xy(0,0,-1,90,30,60);
	while(!read_MANFAN3)
		go_to_xy(-1,10,0,90,30,60);
	
	runActionGroup(224, 1); 
	delay_ms(5000);
}

void GoBack(void)
{
	//runActionGroup(0, 1);   // 复位机械臂
	//delay_ms(5000);
	
}
	
void jieti222(void)
{
	////阶梯单独调试时机械臂的缓冲动作
	 runActionGroup(0, 1); //机械臂准备delay_ms(2500);	
	 delay_ms(4000);	
	////////////////
	
	runActionGroup(190, 1); //机械臂准备	
	delay_ms(3000);	 
	 
	//x距离纠正
	while(1)                                  
	{
		float dis = read_dis1();                  
		if(abs((int)dis-188)<5)break;
		else{
			go_to_xy(0,90,(dis-188)/15,90,20,60);
		}
	}

	
	int i=0;
	i = read_MANFAN2;
	x_set_speed_flag = 1;
	my_car.v_x = 3;
	while(i == read_MANFAN2);
	my_car.v_x = 0;
	x_set_speed_flag = 0;
	
	
	/////////
	go_to_xy(-8,90,0,90,60,60);        //漫反射找到阶梯边缘后，往左边走到第一个物块前
	go_to_xy(0.5,20,0,20,40,60); 
	runActionGroup(110, 1);delay_ms(3000);   //架好机械臂准备扫描第一个积木块
	
	delay_ms(1000);

	if(1)
	{
		int i=0;
		for(i=0;i<8;i++)
		{
			////轮到第3个积木块时，要预先抬高，以免在下面的移动中碰到阶梯
			if(i == 2){runActionGroup(114, 1);delay_ms(1800);}   //升机械臂
//			//if(i == 6){runActionGroup(17, 1);delay_ms(1800);}   //降机械臂
//
			if( i == 2)
			{
				go_to_xy(-13,20,0,20,40,60);
				//delay_ms(2000);
			}
			else if(i == 6)
			{
				go_to_xy(-12,20,0,20,40,60);
				//delay_ms(2000);
			}
			else if(i)
			{
				go_to_xy(-9.5,20,0,20,40,60);               //	走向下一个物块 
			}
			
		while(1)                                    //x距离纠正
		{
			float dis = read_dis1();                  
			if(abs((int)dis-188)<5)break;
			else{
				go_to_xy(0,90,(dis-188)/15,90,20,60);
			}
		}
			//微调识别位置
			switch(i)
			{
				//阶梯前
				case 1:go_to_xy(0.8,20,0,20,40,60); break;
				//阶梯中
				case 2:break;
				case 3:break;
				case 4:break;
				case 5:break;
				//阶梯后
				case 6:go_to_xy(0,20,0,20,40,60); break;
				case 7:go_to_xy(0,20,0,20,40,60); break;
			
			}
//			//纠正角度
//		my_car.target_yaw = 270;
//		while(!(abs(10*(my_car.yaw-270))<5));
				 ////抬高后更新扫描的机械臂动作
			  if(i == 2){runActionGroup(115, 1);delay_ms(1800);}   
    		if(i == 6){runActionGroup(119, 1);delay_ms(1800);}   
						

			MVData[0]=0;
			Usart_SendByte( UART5, 0x31);                       //发送1，openmv开始识别，直到mv返回值
			while(!MVData[0]);                                 //等待mv端发送非0数
			
			if(MVData[0] == 0x33)                             //接收到3，颜色方块
			{
					
					//机械臂准备夹取物块
					if(i<2){runActionGroup(111, 1);delay_ms(3500);}            
					else if(i>=2&&i<6){runActionGroup(116, 1);delay_ms(3500);} 
					else if(i>=6){runActionGroup(120, 1);delay_ms(3500);}

					
					Usart_SendByte( UART5, 0x33);							  //32告诉mv可以夹了
					MVData[0]=0;
					while(MVData[0] != 0x31);								  //32等待mv已经夹好了的信息
					

					///放积木块到小仓库，即移动机械爪到小仓库上方
					if(i<2){runActionGroup(112, 1);delay_ms(4000);}            
					else if(i>=2&&i<6){runActionGroup(117, 1);delay_ms(4000);} 
					else if(i>=6){runActionGroup(121, 1);delay_ms(4000);}     
				
					delay_ms(500);
					Usart_SendByte( UART5, 0x32);				//告诉mv可以松开了
					MVData[0]=0;
					while(MVData[0] != 0x31);					//等待mv已经松开的消息
					
					//复位机械臂，回到扫描状态，准备下一次夹取
					if(i<2){runActionGroup(113, 1);delay_ms(3000);runActionGroup(110, 1);delay_ms(3500);}             
					else if(i>=2 &&i<6){runActionGroup(118, 1);delay_ms(3000);runActionGroup(115, 1);delay_ms(3500);} 
					else if(i>=6){runActionGroup(122, 1);delay_ms(3000);runActionGroup(119, 1);delay_ms(3500);}       
			}
			else if(MVData[0] == 0x34)//不可以夹
			{
				
			}
			else if(MVData[0] == 0x35)//圆环
			{
				u8g2_ClearBuffer(&u8g2);
				u8g2_SetFont(&u8g2,u8g2_font_ncenB08_tf);
				u8g2_DrawStr(&u8g2, 0,50,"yuanhuan_get");
				u8g2_SendBuffer(&u8g2);
					//机械臂准备夹取物块
					if(i<2){go_to_xy(-0.8,20,-0.5,20,40,60);runActionGroup(111, 1);delay_ms(4000);}            
					else if(i>=2&&i<6){go_to_xy(-0.5,20,0,20,40,60);runActionGroup(184, 1);delay_ms(4000);} 
					else if(i>=6){go_to_xy(0.5,20,-0.3,20,40,60);runActionGroup(185, 1);delay_ms(4000);}

					
					Usart_SendByte( UART5, 0x33);							  //32告诉mv可以夹了
					MVData[0]=0;
					while(MVData[0] != 0x31);								  //32等待mv已经夹好了的信息
					

					////////////////////////////////////放圆环放到码垛上
					if(i<2){runActionGroup(180, 1);delay_ms(4000);}            
					else if(i>=2&&i<6){runActionGroup(180, 1);delay_ms(4000);} 
					else if(i>=6){runActionGroup(180, 1);delay_ms(4000);}     
				
					delay_ms(1200);
					Usart_SendByte( UART5, 0x32);				//告诉mv可以松开了
					MVData[0]=0;
					while(MVData[0] != 0x31);					//等待mv已经松开的消息
					delay_ms(500);
					
					
					///////////////////////////////////复位机械臂，回到扫描状态，准备下一次夹取
					if(i<2){runActionGroup(181, 1);delay_ms(5000);runActionGroup(182, 1);delay_ms(3500);}             
					else if(i>=2 &&i<6){runActionGroup(181, 1);delay_ms(5000);runActionGroup(183, 1);delay_ms(3500);} 
					else if(i>=6){runActionGroup(181, 1);delay_ms(5500);runActionGroup(119, 1);delay_ms(3500);}      
			} 
			
		}
		Usart_SendByte( UART5, 0x32);							  //32告诉mv结束阶梯了
	}
}	
void jieti2223(void)
{
	
	runActionGroup(115, 1); //方槽提前摆好	
	delay_ms(1500);	 
	 
	//y距离纠正
	while(1)                                  
	{
		float dis = read_dis1();                  
		if(abs((int)dis-130)<5)break;//115可以，但是怕太极限，达不到然后一直往前，留一点松动（原来是117），130不能再多，否则会砸到方块
		else{
			go_to_xy(0,20,(dis-130)/15,20,20,20);//212有点远，要在一开始贴近，于是197也太多了，135为了给视觉创造良好的识别环境
			
//		char buffer[100];
//		sprintf(buffer, "qian:%5.2f\r\n", dis);
//		Usart_SendString(UART5, buffer);
			
		}
	}


	//往左边走至阶梯边缘
	//方案一：用测距定位(适用于阶梯后有障碍物挡住)
//	while(1){
//		float dis = read_dis1();
//		if(dis < 95){
//			break;
//		}else{
//			go_to_xy(-2,40,0,100,20,100);
//			char buffer[100];
//		    sprintf(buffer, "qian:%5.2f\r\n", dis);
//		    Usart_SendString(UART5, buffer);
//		}
//	}
	
	while(1){
		float dis = read_dis1();
//		if(dis < 105 || dis > 160){                                     //若阶梯边沿有距离很近的障碍物，则启用该方案，否则95
//		if(dis < 95){
		if(dis > 200){		//场地有铁架子，能够被正常测量距离，如果前方扇形区域一点东西都没有，会出现误测导致停不下来
			break;
		}else{
			go_to_xy(-2,40,0,100,40,100);
//			char buffer[100];
//		    sprintf(buffer, "qian:%5.2f\r\n", dis);
//		    Usart_SendString(UART5, buffer);
		}
	}
	go_to_xy(13,100,0,100,100,100);
	go_to_xy(-1,20,0,20,20,20);

	//方案二：用激光定位（适用于阶梯后无障碍物挡住）
//	int i=0;
//	i = read_MANFAN2;
//	x_set_speed_flag = 1;
//	my_car.v_x = -10;
//	while(i == read_MANFAN2);
//	my_car.v_x = 0;
//	x_set_speed_flag = 0;
//	go_to_xy(6.5,100,0,100,100,100);			//找到阶梯边缘后，往右边走到第一个物块前
			
	go_to_xy(0,100,-4,100,100,100);				//先往后再摆动作，最后测距靠近
	runActionGroup(109, 1);//高阶梯识别（已张开）    2700ms		3600ms
	delay_ms(4000);
	
	while(1){
		float dis = read_dis1();                  
		if(abs((int)dis-130)<5)break;//115可以，但是怕太极限，达不到然后一直往前，留一点松动
		else{
			go_to_xy(0,20,(dis-130)/15,20,20,20);//212有点远，要在一开始贴近，于是197也太多了
		}
	}
	
	
	if(1)
	{
		
		int i=0;//i代表第i个物块，0、1是矮阶梯；2、3、4、5是高阶梯；6、7是中阶梯	
//		runActionGroup(109, 1);//高阶梯识别（已张开）    2700ms
//		delay_ms(3000);
		for(i=0;i<8;i++)
		{
			//y距离纠正
				while(1)                                  
				{
					float dis = read_dis1();                  
					if(abs((int)dis-130)<5)break;
					else{
						go_to_xy(0,20,(dis-130)/15,20,20,20);//212有点远，要在一开始贴近，于是197也太多了
					}
				}
			
			MVData[0] = '0';//刷新
			
			/////////////////////矮阶梯部分//////////////////////
			if(i < 2){
				
				
				Usart_SendByte( UART5, 0x41);//开始识别
				
				while(MVData[0] == '0');
				
				if(MVData[0] == 0x42 && i != 1){//不切换阶梯
				
					runActionGroup(121, 1);//矮阶梯抓放回			8700ms		11400ms
					delay_ms(10000);
					
					go_to_xy(8.1,20,0,20,20,20);//走向下一个物块
				
				}
				if(MVData[0] == 0x42 && i == 1){//切换阶梯
				
//					if(rb_t==1){//红
//					my_car.target_yaw = 88;
//					while(!(abs(10*(my_car.yaw-88))<5));
//					}
//					
//					if(rb_t==2){//蓝
//					my_car.target_yaw = 268;
//					while(!(abs(10*(my_car.yaw-268))<5));
//					}
					
					runActionGroup(121, 1);//矮阶梯抓放回			8700ms		11400ms
					delay_ms(10000);
					
					go_to_xy(9.6,20,0,20,20,20);//走向下一个阶梯
				
				}
				if(MVData[0] == 0x43 && i != 1){//不切换阶梯
				
					go_to_xy(8.1,20,0,20,20,20);//走向下一个物块
				
				}
				if(MVData[0] == 0x43 && i == 1){//切换阶梯
					
//					if(rb_t==1){//红
//					my_car.target_yaw = 88;
//					while(!(abs(10*(my_car.yaw-88))<5));
//					}
//					
//					if(rb_t==2){//蓝
//					my_car.target_yaw = 268;
//					while(!(abs(10*(my_car.yaw-268))<5));
//					}
				
					go_to_xy(9.6,20,0,20,20,20);//走向下一个阶梯
				
				}
				
			}
			
			/////////////////////高阶梯部分//////////////////////
			if(i > 1 &&i < 6){
				
				
				Usart_SendByte( UART5, 0x41);//开始识别
				
				while(MVData[0] == '0');
				
				if(MVData[0] == 0x42 && i != 5){//不切换阶梯
				
					
					runActionGroup(123, 1);//高阶梯抓放回		9600ms		12600ms
					delay_ms(11000);
					
					go_to_xy(8.1,20,0,20,20,20);//走向下一个物块
				
				}
				if(MVData[0] == 0x42 && i == 5){//切换阶梯
						
					runActionGroup(123, 1);//高阶梯抓放回		9600ms		12600ms
					delay_ms(11000);
					
					go_to_xy(9.6,20,0,20,20,20);//走向下一个阶梯
				
				}
				if(MVData[0] == 0x43 && i != 5){//不切换阶梯
				
					go_to_xy(8.1,20,0,20,20,20);//走向下一个物块
				
				}
				if(MVData[0] == 0x43 && i == 5){//切换阶梯
					
					go_to_xy(9.6,20,0,20,20,20);//走向下一个阶梯
				
				}
				
			}
			
			/////////////////////中阶梯部分//////////////////////
			if(i > 5 && i < 8){
				
				
				Usart_SendByte( UART5, 0x41);//开始识别
				
				while(MVData[0] == '0');
				
				if(MVData[0] == 0x42 && i != 7){//未离开阶梯
				
					
					runActionGroup(125, 1);//中阶梯抓放回物块		8700ms		11400ms
					delay_ms(10000);
					
					go_to_xy(8.1,20,0,20,20,20);//走向下一个物块
				
				}
				if(MVData[0] == 0x42 && i == 7){//离开阶梯
					
//					my_car.target_yaw = 90;
//					while(!(abs(10*(my_car.yaw-90))<5));
				
					
					runActionGroup(125, 1);//中阶梯抓放回		8700ms		11400ms
					delay_ms(10000);

					go_to_xy(20,100,0,100,100,100);//往右再退					
//					runActionGroup(11, 1);		//方槽复位
//					delay_ms(1300);
					runActionGroup(10, 1);//机械臂复位
				
				}
				if(MVData[0] == 0x43 && i != 7){//未离开阶梯
				
					go_to_xy(8.1,20,0,20,20,20);//走向下一个物块
				
				}
				if(MVData[0] == 0x43 && i == 7){//离开阶梯
					
//					my_car.target_yaw = 90;
//					while(!(abs(10*(my_car.yaw-90))<5));
				
					go_to_xy(20,100,0,100,100,100);//往右再退					
//					runActionGroup(11, 1);		//方槽复位
//					delay_ms(1300);
					runActionGroup(10, 1);//机械臂复位
				}
				
			}
			
		}
	}
	
	Usart_SendByte( UART5, 0x44);//告诉k230结束
	
	/*暂时注释
	if(1)
	{
		int i=0;//i代表第i个物块
		for(i=0;i<8;i++)
		{
			////轮到第3个积木块时，要预先抬高，以免在下面的移动中碰到阶梯
			if(i == 2){runActionGroup(114, 1);delay_ms(1800);}   //升机械臂
			if( i == 2)//到达这一次的物块需要走的距离不同
			{
				go_to_xy(13,20,0,20,40,60);
				//delay_ms(2000);
			}
			else if(i == 6)
			{
				go_to_xy(12,20,0,0,40,0);
				//delay_ms(2000);
			}
			else if(i)
			{
				go_to_xy(9,20,0,20,40,60);               //	走向下一个物块 
			}
			
		while(1)                                    //x距离纠正
		{
			float dis = read_dis1();                  
			if(abs((int)dis-212)<5)break;
			else{
				go_to_xy(0,90,(dis-212)/15,90,20,60);
			}
		}
			//微调识别位置
			switch(i)
			{
				//阶梯前
				case 1:go_to_xy(-0.8,20,0,20,40,60); break;
				//阶梯中
				case 2:
					switch(rb_t)
					{
						case 1 :my_car.target_yaw = 88;while(!(abs(10*(my_car.yaw-88))<1));break;//在这里将错纠错调整一下角度
						case 2 :my_car.target_yaw = 266;while(!(abs(10*(my_car.yaw-266))<1));break;//在这里将错纠错调整一下角度
					}
				case 3:break;
				case 4:break;
				case 5:break;
				//阶梯后
				case 6:go_to_xy(0,20,0,20,40,60); break;
				case 7:go_to_xy(0,20,0,20,40,60); break;
			
			}
//			//纠正角度
//		my_car.target_yaw = 270;
//		while(!(abs(10*(my_car.yaw-270))<5));
				 ////抬高后更新扫描的机械臂动作
			  if(i == 2){runActionGroup(115, 1);delay_ms(1800);}   
    		if(i == 6){runActionGroup(119, 1);delay_ms(1800);}   


			MVData[0]=0;
			Usart_SendByte( UART5, 0x31);                       //发送1，openmv开始识别，直到mv返回值
			while(!MVData[0]);                                 //等待mv端发送非0数
			
			if(MVData[0] == 0x33)                             //接收到3，颜色方块
			{
					
					//机械臂准备夹取物块
					if(i<2){runActionGroup(111, 1);delay_ms(3500);}            
					else if(i>=2&&i<6){runActionGroup(116, 1);delay_ms(3500);} 
					else if(i>=6){runActionGroup(120, 1);delay_ms(3500);}

					
					Usart_SendByte( UART5, 0x33);							  //32告诉mv可以夹了
					MVData[0]=0;
					while(MVData[0] != 0x31);								  //32等待mv已经夹好了的信息
					

					///放积木块到小仓库，即移动机械爪到小仓库上方
					if(i<2){runActionGroup(112, 1);delay_ms(4000);}            
					else if(i>=2&&i<6){runActionGroup(117, 1);delay_ms(4000);} 
					else if(i>=6){runActionGroup(121, 1);delay_ms(4000);}     
				
					delay_ms(500);
					Usart_SendByte( UART5, 0x32);				//告诉mv可以松开了
					MVData[0]=0;
					while(MVData[0] != 0x31);					//等待mv已经松开的消息
					
					//复位机械臂，回到扫描状态，准备下一次夹取
					if(i<2){runActionGroup(113, 1);delay_ms(3000);runActionGroup(110, 1);delay_ms(3500);}             
					else if(i>=2 &&i<6){runActionGroup(118, 1);delay_ms(3000);runActionGroup(115, 1);delay_ms(3500);} 
					else if(i>=6){runActionGroup(122, 1);delay_ms(3000);runActionGroup(119, 1);delay_ms(3500);}       
			}
			else if(MVData[0] == 0x34)//不可以夹
			{
				
			}
			else if(MVData[0] == 0x35)//圆环
			{
				u8g2_ClearBuffer(&u8g2);
				u8g2_SetFont(&u8g2,u8g2_font_ncenB08_tf);
				u8g2_DrawStr(&u8g2, 0,50,"yuanhuan_get");
				u8g2_SendBuffer(&u8g2);
					//机械臂准备夹取物块
					if(i<2){go_to_xy(0,20,-0.5,20,40,60);runActionGroup(111, 1);delay_ms(4000);}            
					else if(i>=2&&i<6){go_to_xy(0,20,0,20,40,60);runActionGroup(184, 1);delay_ms(4000);} 
					else if(i>=6){go_to_xy(0,20,-0.3,20,40,60);runActionGroup(185, 1);delay_ms(4000);}

					
					Usart_SendByte( UART5, 0x33);							  //32告诉mv可以夹了
					MVData[0]=0;
					while(MVData[0] != 0x31);								  //32等待mv已经夹好了的信息
					

					////////////////////////////////////放圆环放到码垛上
					if(i<2){runActionGroup(180, 1);delay_ms(4000);}            
					else if(i>=2&&i<6){runActionGroup(180, 1);delay_ms(4000);} 
					else if(i>=6){runActionGroup(180, 1);delay_ms(4000);}     
				  
					delay_ms(1800);
					Usart_SendByte( UART5, 0x32);				//告诉mv可以松开了
					MVData[0]=0;
					while(MVData[0] != 0x31);					//等待mv已经松开的消息
					delay_ms(500);
					
					
					///////////////////////////////////复位机械臂，回到扫描状态，准备下一次夹取
					if(i<2){runActionGroup(181, 1);delay_ms(5000);runActionGroup(182, 1);delay_ms(3500);}             
					else if(i>=2 &&i<6){runActionGroup(181, 1);delay_ms(5000);runActionGroup(183, 1);delay_ms(3500);} 
					else if(i>=6){runActionGroup(181, 1);delay_ms(5500);runActionGroup(119, 1);delay_ms(3500);}      
			} 
			
		}
		Usart_SendByte( UART5, 0x32);							  //32告诉mv结束阶梯了
		
	}
	*/
	

}

void jieti2221(void)
{
	////阶梯单独调试时机械臂的缓冲动作
	 runActionGroup(0, 1); //机械臂准备delay_ms(2500);	
	 delay_ms(5000);	
	////////////////
	
	runActionGroup(190, 1); //机械臂准备	
	delay_ms(2500);	 
	 
	//x距离纠正
	while(1)                                  
	{
		float dis = read_dis1();                  
		if(abs((int)dis-188)<5)break;
		else{
			go_to_xy(0,90,(dis-188)/15,90,20,60);
		}
	}

	
	  int i=0;
	i = read_MANFAN2;
	x_set_speed_flag = 1;
	my_car.v_x = -3;
	while(i == read_MANFAN2);
	my_car.v_x = 0;
	x_set_speed_flag = 0;
	
	
	/////////
	go_to_xy(8,90,0,90,60,60);        //漫反射找到阶梯边缘后，往右边走到第一个物块前

	runActionGroup(110, 1);delay_ms(3000);   //架好机械臂准备扫描第一个积木块
	
	delay_ms(1000);

	if(1)
	{
		int i=0;
		for(i=0;i<8;i++)
		{
			////轮到第3个积木块时，要预先抬高，以免在下面的移动中碰到阶梯
			if(i == 2){runActionGroup(114, 1);delay_ms(1800);}   //升机械臂
//			//if(i == 6){runActionGroup(17, 1);delay_ms(1800);}   //降机械臂
//
			if( i == 2)
			{
				go_to_xy(13,20,0,20,40,60);
				//delay_ms(2000);
			}
			else if(i == 6)
			{
				go_to_xy(12,20,0,20,40,60);
				//delay_ms(2000);
			}
			else if(i)
			{
				go_to_xy(9,20,0,20,40,60);               //	走向下一个物块 
				//delay_ms(2000);
			}
			
		while(1)                                    //x距离纠正
		{
			float dis = read_dis1();                  
			if(abs((int)dis-188)<5)break;
			else{
				go_to_xy(0,90,(dis-188)/15,90,20,60);
			}
		}

				 ////抬高后更新扫描的机械臂动作
			  if(i == 2){runActionGroup(115, 1);delay_ms(1800);}   
    		if(i == 6){runActionGroup(119, 1);delay_ms(1800);}   
						

			
			Usart_SendByte( UART5, 0x31);                       //发送1，openmv开始识别，直到mv返回值
			MVData[0]=0;
			while(!MVData[0]);                                 //等待mv端发送非0数
			
			if(MVData[0] == 0x33)                             //接收到3，可夹取积木块
			{
					
					//机械臂准备夹取物块
					if(i<2){runActionGroup(111, 1);delay_ms(3500);}            
					else if(i>=2&&i<6){runActionGroup(116, 1);delay_ms(3500);} 
					else if(i>=6){runActionGroup(120, 1);delay_ms(3500);}

					
					Usart_SendByte( UART5, 0x33);							  //32告诉mv可以夹了
					MVData[0]=0;
					while(MVData[0] != 0x31);								  //32等待mv已经夹好了的信息
					

					///放积木块到小仓库，即移动机械爪到小仓库上方
					if(i<2){runActionGroup(112, 1);delay_ms(4000);}            
					else if(i>=2&&i<6){runActionGroup(117, 1);delay_ms(4000);} 
					else if(i>=6){runActionGroup(121, 1);delay_ms(4000);}     
				
					delay_ms(500);
					Usart_SendByte( UART5, 0x32);				//告诉mv可以松开了
					MVData[0]=0;
					while(MVData[0] != 0x31);					//等待mv已经松开的消息
					
					//复位机械臂，回到扫描状态，准备下一次夹取
					if(i<2){runActionGroup(113, 1);delay_ms(3000);runActionGroup(110, 1);delay_ms(3500);}             
					else if(i>=2 &&i<6){runActionGroup(118, 1);delay_ms(3000);runActionGroup(115, 1);delay_ms(3500);} 
					else if(i>=6){runActionGroup(122, 1);delay_ms(3000);runActionGroup(119, 1);delay_ms(3500);}       
			}
			else if(MVData[0] == 0x34)//不可以夹
			{
				;
			}
			else if(MVData[0] == 0x35)//圆环
			{
				//机械臂准备夹取物块
					if(i<2){runActionGroup(111, 1);delay_ms(3500);}         ///调过了不知道可不可以   
					else if(i>=2&&i<6){runActionGroup(116, 1);delay_ms(3500);}   ////重调一个动作组
					else if(i>=6){runActionGroup(120, 1);delay_ms(3500);}///////这个可以不用管

					
					Usart_SendByte( UART5, 0x33);							  //32告诉mv可以夹了
					MVData[0]=0;
					while(MVData[0] != 0x31);								  //32等待mv已经夹好了的信息
					

					///放圆环到柱子上方/////////此处
					if(i<2){runActionGroup(112, 1);delay_ms(4000);}            
					else if(i>=2&&i<6){runActionGroup(117, 1);delay_ms(4000);} 
					else if(i>=6){runActionGroup(121, 1);delay_ms(4000);}     
				
					delay_ms(500);
					Usart_SendByte( UART5, 0x32);				//告诉mv可以松开了
					MVData[0]=0;
					while(MVData[0] != 0x31);					//等待mv已经松开的消息
					
					//复位机械臂，回到扫描状态，准备下一次夹取/////////////////////////////////////此处要调
					if(i<2){runActionGroup(113, 1);delay_ms(3000);runActionGroup(110, 1);delay_ms(3500);}             
					else if(i>=2 &&i<6){runActionGroup(118, 1);delay_ms(3000);runActionGroup(115, 1);delay_ms(3500);} 
					else if(i>=6){runActionGroup(122, 1);delay_ms(3000);runActionGroup(119, 1);delay_ms(3500);}       
			
			} 
			
		}
		Usart_SendByte( UART5, 0x32);							  //32告诉mv结束阶梯了
	}
}		
	
	
	
	
	
	
	
	
	
	
//	
//	//Usart_SendByte( UART5, 1);
///*------------------------创建任务-------------------------*/
//  xReturn1 = xTaskCreate((TaskFunction_t )Task1,                           /* 任务入口函数 */
//                        (const char*    )"Task1",                         /* 任务名字 */
//                        (uint16_t       )1024*2 ,                             /* 任务栈大小 */
//                        (void*          )NULL,                            /* 任务入口函数参数 */
//                        (UBaseType_t    )2,                               /* 任务的优先级 */
//                        (TaskHandle_t*  )&Task1_Handle);                  /* 任务控制块指针 */ 

//  xReturn2 = xTaskCreate((TaskFunction_t )Task2,                           /* 任务入口函数 */
//                        (const char*    )"Task2",                         /* 任务名字 */
//                        (uint16_t       )1024*2,                             /* 任务栈大小 */
//                        (void*          )NULL,	                          /* 任务入口函数参数 */
//                        (UBaseType_t    )1,	                              /* 任务的优先级 */
//                        (TaskHandle_t*  )&Task2_Handle);                  /* 任务控制块指针 */

//  xReturn3 = xTaskCreate((TaskFunction_t )Task3,                           /* 任务入口函数 */
//                        (const char*    )"Task3",                         /* 任务名字 */
//                        (uint16_t       )1024*2,                             /* 任务栈大小 */
//                        (void*          )NULL,	                          /* 任务入口函数参数 */
//                        (UBaseType_t    )1,	                              /* 任务的优先级 */
//                        (TaskHandle_t*  )&Task3_Handle);                  /* 任务控制块指针 */
//												
//	xReturn4 = xTaskCreate((TaskFunction_t )Task4,                           /* 任务入口函数 */
//                        (const char*    )"Task4",                         /* 任务名字 */
//                        (uint16_t       )1024*2,                             /* 任务栈大小 */
//                        (void*          )NULL,	                          /* 任务入口函数参数 */
//                        (UBaseType_t    )0,	                              /* 任务的优先级 */
//                        (TaskHandle_t*  )&Task4_Handle);                  /* 任务控制块指针 */
///*------------------------启动任务-------------------------*/
//  if((pdPASS == xReturn1) && (pdPASS == xReturn2)&&(pdPASS == xReturn3)&&(pdPASS == xReturn4))
//	{
//    vTaskStartScheduler();                                                /* 启动任务调度 */  /* 启动任务，开启调度 */
//		printf("okk0");
//	}
//  else
//	{
//		printf("off0");
//    return -1;  
//	}
//  
//  while(1);                                                               /* 正常不会执行到这里 */    
//}
/**************************************************************************
FreeRTos:
暂停任务：vTaskSuspend(TaskHandle);如果暂停自己则参数设置为NULL
任务退出停止状态：vTaskResume(TaskHandle);
进入阻塞状态：
    延时函数：vTaskDelay();定时单位为系统定时器的一个Tick，从执行函数开始休眠一段时间
    周期执行函数：vTaskDelayUntil(*t,detal_t);定时单位为系统定时器的一个Tick,每次运行间隔固定detal_t,
    下一次执行的时间为*t+detal_t，并且t变量传进来的是指针，执行完函数会自动更新t的值加上delta_t
获取系统时间：TickType_t t=xTaskGetTickCount();表示经过系统定时器的中断次数
删除任务： vTaskDelete(TaskHandle);删除自己传入NULL，但自杀后无法清理内存，
需要空闲任务帮忙清理，因此需要最高优先级为0，让空闲任务有机会执行，如果是别的任务删除该任务，
则不需要空闲任务清理
**************************************************************************/

/**********************************************************************
  * @ 函数名  ： LED_Task
  * @ 功能说明： LED_Task任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/
static void Task1(void* parameter)
{	
    //Task1_main();
	//while(1);
	    while (1)
    {
			//printf("1\n");
			//vTaskDelay(200);
			Usart_SendByte( UART5, 1);
//			while(MVData[0]==0);
			while(HongWai   ==0);										//等接收货物
			Task_TIM_NewState(ENABLE);
			vTaskSuspend(NULL);
	
    }
}

/**********************************************************************
  * @ 函数名  ： LED_Task
  * @ 功能说明： LED_Task任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/
static void Task2(void* parameter)
{	
    //Task2_main();
//	uint8_t d=0;
    while (1)
    {
////			printf("2\n");
////			vTaskDelay(200);
//			d=HuiDuOUT1<<7|HuiDuOUT2<<6|HuiDuOUT3<<5|HuiDuOUT4<<4|HuiDuOUT5<<3|HuiDuOUT6<<2|HuiDuOUT7<<1|HuiDuOUT8;
//	    Usart_SendByte( USART1, (uint8_t)d);
//			vTaskDelay(200);
    }
}

/**********************************************************************
  * @ 函数名  ： LED_Task
  * @ 功能说明： LED_Task任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/
static void Task3(void* parameter)
{	
   // Task3_main();
	while(1);
}

/**********************************************************************
  * @ 函数名  ： LED_Task
  * @ 功能说明： LED_Task任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/
static void Task4(void* parameter)
{	
   // Task3_main();
	while(1);
}

/********************************END OF FILE****************************/
