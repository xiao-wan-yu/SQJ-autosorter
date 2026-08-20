#include "encoder.h"
#include "stm32f10x_tim.h"



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


volatile int encoder1_num=0;
volatile int encoder2_num=0;
volatile int encoder3_num=0;
volatile int encoder4_num=0;
/**************************************************************************
函数功能：把TIM2初始化为编码器接口模式
入口参数：无
返回  值：无
说明    ：1、编码器接线：PA15、PB3
          2、TIM2默认输入引脚为PA0、PA1，与TIM5冲突，因此复用到PA15、PB3
					3、编码器转动一圈输出16个脉冲，定时器采用四倍频计数，故编码器转动一圈定时器计数64
					4、电机减速比为80，因此电机转动一圈，定时器计数5120个数，自动重装载值设置为（5120-1）
					   时，电机转动一圈产生一次中断。
**************************************************************************/
void TIM2_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStruct;
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStruct.NVIC_IRQChannel=TIM2_IRQn;                         //配置中断源
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=1;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	
	NVIC_Init(&NVIC_InitStruct);
}
void Encoder_Init_TIM2(void)
{
/*---------------------------IO口配置------------------------------*/
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);              //开启GPIO时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	                       //端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;              //浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);					                   //根据设定参数初始化GPIO
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);              //开启GPIO时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	                         //端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;              //浮空输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);					                   //根据设定参数初始化GPIO
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);               //开启复用功能寄存器时钟
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);           //将PA15、PB3、PB4配置为普通IO口，默认为JTAG调试功能，不配置为普通引脚无法工作
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2,ENABLE);               //将通道1、2的引脚复用到PA15、PB3中，通道3、4不变为PA2、PA3
	//GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2,ENABLE);             //将通道3、4的引脚复用到PA10、PB11中，通道1、2不变为PA0、PA1
	//GPIO_PinRemapConfig(GPIO_FullRemap_TIM2,ENABLE);                 //将通道1、2、3、4的引脚复用到PA15、PB3、PA10、PB11中
	
	
	/*--------------------时基结构体初始化-------------------------*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);               //使能定时器4的时钟,除高级定时器是APB2，其他都是APB1
	
	TIM_DeInit(TIM2);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);                    //清除配置
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;                         // 驱动CNT计数器的时钟 = Fck_int/(psc+1),计数频率不分频，即72MHz
  TIM_TimeBaseStructure.TIM_Period = (5120-1);                          //设定计数器自动重装值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;            //选择时钟分频：不分频，时钟分频因子 ，配置死区时间时需要用到
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;        //TIM向上计数  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, \
	TIM_ICPolarity_BothEdge, TIM_ICPolarity_BothEdge);                 //使用编码器模式3，四倍频


  TIM_ICInitTypeDef TIM_ICInitStructure;  
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 15;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);
	
	
  
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);                              //清除TIM的更新标志位
  TIM2_NVIC_Config();                                                //配置中断优先级
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);                         //开启更新中断
  //Reset counter
  TIM_SetCounter(TIM2,0);                                            //重置计数器
  TIM_Cmd(TIM2, ENABLE);                                             //开启定时器
}
/**************************************************************************
函数功能：把TIM3初始化为编码器接口模式
入口参数：无
返回  值：无
说明    ：1、编码器接线：PB4\PB5
          2、TIM2默认输入引脚为PA6、PA7，与TIM5冲突，因此复用到PB4、PB5
					3、编码器转动一圈输出16个脉冲，定时器采用四倍频计数，故编码器转动一圈定时器计数64
					4、电机减速比为80，因此电机转动一圈，定时器计数5120个数，自动重装载值设置为（5120-1）
					   时，电机转动一圈产生一次中断。
**************************************************************************/
void TIM3_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStruct;
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStruct.NVIC_IRQChannel=TIM3_IRQn;                         //配置中断源
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=1;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	
	NVIC_Init(&NVIC_InitStruct);
}
void Encoder_Init_TIM3(void)
{
/*---------------------------IO口配置------------------------------*/
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);              //开启GPIO时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;	             //端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;              //浮空输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);					                   //根据设定参数初始化GPIO
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);               //开启复用功能寄存器时钟
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3,ENABLE);                //将定时器3的CH1、CH2重定义到PB4、PB5，CH3、CH4默认为PB0、PB1
	//GPIO_PinRemapConfig(GPIO_FullRemap_TIM3,ENABLE);                 //将定时器3的CH1、CH2、CH3、CH4重定义到PC6、PC7、PC8、PC9
	/*--------------------时基结构体初始化-------------------------*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);               //使能定时器4的时钟,除高级定时器是APB2，其他都是APB1
	
	TIM_DeInit(TIM3);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;                         // 驱动CNT计数器的时钟 = Fck_int/(psc+1),计数频率不分频，即72MHz
  TIM_TimeBaseStructure.TIM_Period = (5120-1);                       //设定计数器自动重装值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;            //选择时钟分频：不分频，时钟分频因子 ，配置死区时间时需要用到
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;        //TIM向上计数  
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, \
	TIM_ICPolarity_BothEdge, TIM_ICPolarity_BothEdge);                 //使用编码器模式3


  TIM_ICInitTypeDef TIM_ICInitStructure;  
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 15;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
	
  
  TIM_ClearFlag(TIM3, TIM_FLAG_Update);                              //清除TIM的更新标志位
  TIM3_NVIC_Config();                                                //配置中断优先级
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);                         //开启更新中断
  //Reset counter
  TIM_SetCounter(TIM3,0);                                            //重置计数器
  TIM_Cmd(TIM3, ENABLE);                                             //开启定时器 

}

/**************************************************************************
函数功能：把TIM4初始化为编码器接口模式
入口参数：无
返回  值：无
说明    ：1、编码器接线：PB6、PB7
					3、编码器转动一圈输出16个脉冲，定时器采用四倍频计数，故编码器转动一圈定时器计数64
					4、电机减速比为80，因此电机转动一圈，定时器计数5120个数，自动重装载值设置为（5120-1）
					   时，电机转动一圈产生一次中断。
**************************************************************************/
void TIM4_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStruct;
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStruct.NVIC_IRQChannel=TIM4_IRQn;                         //配置中断源
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=1;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	
	NVIC_Init(&NVIC_InitStruct);
}
void Encoder_Init_TIM4(void)
{
/*---------------------------IO口配置------------------------------*/
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);              //开启GPIO时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	             //端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;              //浮空输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);					                   //根据设定参数初始化GPIO
	
	
	/*--------------------时基结构体初始化-------------------------*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);               //使能定时器4的时钟,除高级定时器是APB2，其他都是APB1
	
	TIM_DeInit(TIM4);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;                         // 驱动CNT计数器的时钟 = Fck_int/(psc+1),计数频率不分频，即72MHz
  TIM_TimeBaseStructure.TIM_Period = (5120-1);                       //设定计数器自动重装值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;            //选择时钟分频：不分频，时钟分频因子 ，配置死区时间时需要用到
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;        //TIM向上计数  
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, \
	TIM_ICPolarity_BothEdge, TIM_ICPolarity_BothEdge);                  //使用编码器模式3

  TIM_ICInitTypeDef TIM_ICInitStructure;  
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 15;
  TIM_ICInit(TIM4, &TIM_ICInitStructure);
  
  TIM_ClearFlag(TIM4, TIM_FLAG_Update);                              //清除TIM的更新标志位
  TIM4_NVIC_Config();                                                //配置中断优先级
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);                         //开启更新中断
  //Reset counter
  TIM_SetCounter(TIM4,0);                                            //重置计数器
  TIM_Cmd(TIM4, ENABLE);                                             //开启定时器 

}

/**************************************************************************
函数功能：把TIM5初始化为编码器接口模式
入口参数：无
返回  值：无
说明    ：1、编码器接线：PA0、PA1
					3、编码器转动一圈输出16个脉冲，定时器采用四倍频计数，故编码器转动一圈定时器计数64
					4、电机减速比为80，因此电机转动一圈，定时器计数5120个数，自动重装载值设置为（5120-1）
					   时，电机转动一圈产生一次中断。
**************************************************************************/
void TIM5_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStruct;
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStruct.NVIC_IRQChannel=TIM5_IRQn;                         //配置中断源
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=1;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	
	NVIC_Init(&NVIC_InitStruct);
}
void Encoder_Init_TIM5(void)
{
/*---------------------------IO口配置------------------------------*/
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);              //开启GPIO时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;	             //端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;              //浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);					                   //根据设定参数初始化GPIO
	
	/*--------------------时基结构体初始化-------------------------*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);               //使能定时器4的时钟,除高级定时器是APB2，其他都是APB1
	
	TIM_DeInit(TIM5);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;                         // 驱动CNT计数器的时钟 = Fck_int/(psc+1),计数频率不分频，即72MHz
  TIM_TimeBaseStructure.TIM_Period = (5120-1);                          //设定计数器自动重装值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;            //选择时钟分频：不分频，时钟分频因子 ，配置死区时间时需要用到
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;        //TIM向上计数  
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, \
	TIM_ICPolarity_BothEdge, TIM_ICPolarity_BothEdge);                 //使用编码器模式3

  TIM_ICInitTypeDef TIM_ICInitStructure;  
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 15;
  TIM_ICInit(TIM5, &TIM_ICInitStructure);
  
  TIM_ClearFlag(TIM5, TIM_FLAG_Update);                              //清除TIM的更新标志位
  TIM5_NVIC_Config();                                                //配置中断优先级
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);                         //开启更新中断
  //Reset counter
  TIM_SetCounter(TIM5,0);                                            //重置计数器
  TIM_Cmd(TIM5, ENABLE);                                             //开启定时器 

}


























