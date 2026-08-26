#include "pwm.h"

/**************************************************************************

						定时器			分辨率	计数器类型	预分频系数	产生DMA		捕获/比较通道		互补输出	  时钟     说明
						
基本定时器 	TIM6/7			16位		向上				1-65535			可以					0						没有			  APB1     没有外部GPIO，时钟来源PCLK1
	
通用定时器	TIM2/3/4/5	16位		向上/向下		1-65535			可以					4						没有        APB1

高级定时器	TIM1/8			16位		向上/向下		1-65535			可以					4						有（通道1—3）APB2

**************************************************************************/
/**************************************************************************
                           ZET6/VET6引脚分布
高级定时器                                     通用定时器
       TIM1           TIM8    TIM2       TIM5    TIM3           TIM4
CH1    PA8/PE9        PC6     PA0/PA15   PA0     PA6/PC6/PB4    PB6/PD12
CH1N   PB13/PA7/PE8   PA7      
CH2    PA9/PE11       PC7     PA1/PB3    PA1     PA7/PC7/PB5    PB7/PD13
CH2N   PB14/PB0/PE10  PB0       
CH3    PA10/PE13      PC8     PA2/PB10   PA2     PB0/PC8        PB8/PD14
CH3N   PB15/PB1/PE12  PB1        
CH4    PA11/PE14      PC9     PA3/PB11   PA3     PB1/PC9        PB9/PD15
CH4N   
ETR    PA12/PE7       PA0     PA0/PA15           PD2            PE0
BKIN   PB12/PA6/PE15  PA6     

**************************************************************************/
/**************************************************************************
函数功能：TIM1输出四路PWM
输入变量：无
返回值	：无
说明    ：1、PWM输出引脚：PE9、PE11、PE13、PE14
          PWM信号 周期和占空比的计算
							 ARR ：自动重装载寄存器的值
							 CLK_cnt：计数器的时钟，等于 Fck_int / (psc+1) = 72M/(psc+1)
							 PWM 信号的周期 T = (ARR+1) * (1/CLK_cnt) = (ARR+1)*(PSC+1) / 72M
							 占空比P=CCR/(ARR+1)
**************************************************************************/
void TIM1_PWM_init(void)
{
/*---------------------------IO口配置------------------------------*/
  GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);                     // 输出比较通道 GPIO 初始化
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);                      //开启复用功能寄存器时钟
																																						/*******重映射设置*****************
	                                                                                      默认   部分   全部
																																						TIM1_ETR    PA12   PA12   PE7
																																						TIM1_CH1    PA8    PA8    PE9
																																						TIM1_CH2    PA9    PA9    PE11
																																						TIM1_CH3    PA10   PA10   PE13
																																						TIM1_CH4    PA11   PA11   PE14
																																						TIM1_BKIN   PB12   PA6    PE15
																																						TIM1_CH1N   PB13   PA7    PE8
																																						TIM1_CH2N   PB14   PB0    PE10
																																						TIM1_CH3N   PB15   PB1    PE12
																																						**********************************/
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1,ENABLE);                       //部分重映射
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,ENABLE);                          //全部重映射
	
/*---------------------------寄存器配置------------------------------*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = (800-1); 
	TIM_TimeBaseStructure.TIM_Prescaler =(90-1); 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 

	//必须全部配置，否则无法输出PWM
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_Pulse = 0; 		//占空比
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;//
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;//
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;//
	TIM_OC1Init(TIM1, &TIM_OCInitStructure); //  
 
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);//
	TIM_ARRPreloadConfig(TIM1, ENABLE);//
	

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);  //?
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);  //
    
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);    
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
    
	TIM_Cmd(TIM1, ENABLE);  //
    TIM_CtrlPWMOutputs(TIM1, ENABLE);                             //
    
	  TIM_SetCompare1(TIM1,0);        //
    TIM_SetCompare2(TIM1,0);
    TIM_SetCompare3(TIM1,0);
    TIM_SetCompare4(TIM1,0);
}
static void moter_dir_gpio(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;//初始化结构体
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//根据需要选择输出模式
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;//输出引脚号
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;//配置输出速度，跟功耗有关，速度越高，功耗越高
	GPIO_Init(GPIOD,&GPIO_InitStruct);
}
/**************************************************************************
函数功能：TIM8输出四路PWM
输入变量：无
返回值	：无
说明    ：1、PWM输出引脚PC6、PC7、PC8、PC9
          PWM信号 周期和占空比的计算
							 ARR ：自动重装载寄存器的值
							 CLK_cnt：计数器的时钟，等于 Fck_int / (psc+1) = 72M/(psc+1)
							 PWM 信号的周期 T = (ARR+1) * (1/CLK_cnt) = (ARR+1)*(PSC+1) / 72M
							 占空比P=CCR/(ARR+1)
**************************************************************************/
void TIM8_PWM_init(void)
{
/*---------------------------IO口配置------------------------------*/
  GPIO_InitTypeDef GPIO_InitStructure;

  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);                     // 输出比较通道 GPIO 初始化
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	moter_dir_gpio();
/*---------------------------寄存器配置------------------------------*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = (1000-1); 
	TIM_TimeBaseStructure.TIM_Prescaler =(90-1); 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); 

	//必须全部配置，否则无法输出PWM
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_Pulse = 0; 		//占空比
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;//
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;//
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;//
	TIM_OC1Init(TIM8, &TIM_OCInitStructure); //  
 
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);//
	TIM_ARRPreloadConfig(TIM8, ENABLE);//
	

	TIM_OC1Init(TIM8, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
	
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);  //
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);  //
	
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
	
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);    
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
	
	TIM_Cmd(TIM8, ENABLE);  //?
	TIM_CtrlPWMOutputs(TIM8, ENABLE);
	
	TIM_SetCompare1(TIM8,0);
	TIM_SetCompare2(TIM8,0);
	TIM_SetCompare3(TIM8,0);
	TIM_SetCompare4(TIM8,0);
}





















































