#include "vl53l1x.h"
#include "core_cm3.h"

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
static void baseTime_Config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		
		
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);                  // 开启定时器时钟,即内部时钟CK_INT=72M
	
		TIM_TimeBaseStructure.TIM_Prescaler= 71;                              // 预分频器PSC的分频系数，让计数器计数一次的值为1us
    TIM_TimeBaseStructure.TIM_Period = 65*1000-1;	                          // 自动重装载寄存器的值，即设置计数器最大值，让计数次数为所设置的值加一
    
	  TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);                       // 初始化定时器
		
    //TIM_ClearFlag(TIM7, TIM_FLAG_Update);                                 // 清除计数器中断标志位
    //TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);                              // 计数器计数值达到最大后产生中断
		
    TIM_Cmd(TIM7, ENABLE);	                                              // 使能计数器
}
static void gpio_init()
{
  GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE );  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_Init(GPIOE, &GPIO_InitStructure); 
}
void vl53l1x_init()
{
	baseTime_Config();
	gpio_init();
}
float read_dis1() //测得前面的距离
{
	float temp;
	__disable_irq();//关闭所有中断的响应
	while(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_9));//等待高电平结束
	while(!GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_9));//等待高电平
	TIM_SetCounter(TIM7, 0);
	while(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_9));//等待高电平结束
	temp = ((float)TIM_GetCounter(TIM7)/10.0);
	__enable_irq();
	return temp;
}
float read_dis2() //测得后面的距离
{
	float temp;
	__disable_irq();//关闭所有中断的响应
	while(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_11));//等待高电平结束
	while(!GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_11));//等待高电平
	TIM_SetCounter(TIM7, 0);
	while(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_11));//等待高电平结束
	temp = ((float)TIM_GetCounter(TIM7)/10.0);
	__enable_irq();
	return temp;
}
float read_dis3()
{
	float temp;
	__disable_irq();//关闭所有中断的响应
	while(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_13));//等待高电平结束
	while(!GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_13));//等待高电平
	TIM_SetCounter(TIM7, 0);
	while(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_13));//等待高电平结束
	temp = ((float)TIM_GetCounter(TIM7)/10.0);
	__enable_irq();
	return temp;
}
//TIM_GetCounter(TIM7)
//TIM_SetCounter(TIM7, 0);