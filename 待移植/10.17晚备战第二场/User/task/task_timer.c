#include "task_timer.h"
#include "usart.h"
#include "8LuHuiDu.h"
#include "Moter.h"




/**************************************************************************
函数功能：开启、关闭定时器
输入变量：无
返回值	：无
**************************************************************************/
void Task_TIM_NewState(FunctionalState newstate)
{
		TIM_Cmd(TIM6, newstate);	                                              // 使能计数器
}



/*-------------------------------定时器配置--------------------------------*/

/**************************************************************************
函数功能：配置基本定时器的中断优先级
输入变量：无
返回值	：无
说明    ：更改时需要更改中断源和优先级
**************************************************************************/
static void Task_TIM_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
  
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);                      // 设置中断组为0		
	
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn ;	                     // 设置中断来源
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	             // 设置主优先级为 0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	                   // 设置抢占优先级为3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
    NVIC_Init(&NVIC_InitStructure);
}

/**************************************************************************
函数功能：配置基本定时器
输入变量：无
返回值	：无
说明    ：1、定时器的默认频率都为72MHz，因为默认下主频为72MHz，
                 APB1分频系数为2，APB2分频系数为1
          2、定时器2-7，即基本定时器和通用定时器，其时钟挂载在APB1
						     如果APB1的分频因子为1，则定时器的频率与其一样，否则需要乘以二
					3、定时器1和8，即高级定时器，其时钟挂载在APB2
						     如果APB2的分频因子为1，则定时器的频率与其一样，否则需要乘以二
					4、预分频器PSC对时钟进行分频，分频后的时钟进入计数器
					       计数器的计数时钟频率等于定时器时钟频率/（PSC分频系数+1）
						     分频系数取值为(1,65536)，不包含边界
					5、如果时钟频率为72MHz，设置分频系数为71，则计数器每计数一次的时间为1us
					6、计数器最大计数值为65535，用户可设置自动重装载寄存器设置其最大值，
					       实际计数的值为，自动重装载寄存器的值加一，如果要计数1000次，则设置999

          7、TIM_TimeBaseInitTypeDef结构体里面有5个成员，TIM6和TIM7的寄存器里面只有
                 TIM_Prescaler和TIM_Period，所以使用TIM6和TIM7的时候只需初始化这两个成员即可，
                 另外三个成员是通用定时器和高级定时器才有.
								 |---------------------------------------------------------
								 |typedef struct
								 |{ TIM_Prescaler            都有
								 |	TIM_CounterMode			     TIMx,x[6,7]没有，其他都有
								 |  TIM_Period               都有
								 |  TIM_ClockDivision        TIMx,x[6,7]没有，其他都有
								 |  TIM_RepetitionCounter    TIMx,x[1,8,15,16,17]才有
								 |}TIM_TimeBaseInitTypeDef; 
								 |---------------------------------------------------------

**************************************************************************/
static void Task_TIM_Mode_Config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		
		
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);                  // 开启定时器时钟,即内部时钟CK_INT=72M
	
		TIM_TimeBaseStructure.TIM_Prescaler= 71;                              // 预分频器PSC的分频系数，让计数器计数一次的值为0.5us
    TIM_TimeBaseStructure.TIM_Period = 5-1;	                          // 自动重装载寄存器的值，即设置计数器最大值，让计数次数为所设置的值加一
    
	  TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);                       // 初始化定时器
		
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);                                 // 清除计数器中断标志位
    TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);                              // 计数器计数值达到最大后产生中断
		
    TIM_Cmd(TIM6, DISABLE);	                                              // 使能计数器
}

void Task_TIM_Init(void)
{
	Task_TIM_NVIC_Config();
	Task_TIM_Mode_Config();
}












