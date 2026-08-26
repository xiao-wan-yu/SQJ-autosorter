#include "Moter_ZL.h"
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
                            本工程使用的资源
					定时器：
					          高级定时器TIM1、TIM8用于产生8路PWM信号
										通用定时器TIM2、TIM3、TIM4、TIM5用于采集四个编码器信号
					IO引脚：
					          8路PWM
																TIM1-CH1          PE9
																TIM1-CH2          PE11
																TIM1-CH3          PE13
																TIM1-CH4          PE14
																TIM8-CH1          PC6
																TIM8-CH2          PC7
																TIM8-CH3          PC8
																TIM8-CH4          PC9
										4个编码器   
										            TIM2              CH1---PA15     CH2---PB3
																TIM3              CH1---PB4      CH2---PB5
																TIM4              CH1---PB6      CH2---PB7
																TIM5              CH1---PA0      CH2---PA1
**************************************************************************/
/**************************************************************************
                            PWM、编码器调用接口
					修改PWM占空比：   
					                  TIM_SetCompare1(TIMx,num);               //库函数
														     TIMx ：可选TIM1、TIM8
																 num  ：占空比可选0-800
					读取编码器定时器的计数值，即未满一圈的值：
					                  TIM_GetCounter(TIMx);                    //库函数
														     TIMx : 可选TIM2、TIM3、TIM4、TIM5
					读取编码器总计数值：
					                  TIM_GetAllCounter(TIMx);                 //自定义函数
														     TIMx : 可选TIM2、TIM3、TIM4、TIM5
																 
**************************************************************************/
struct CAR car;
extern volatile float angle;                                                      //陀螺仪实时角度
void moter_init()
{
	car.moter_en = 1;
	car.act_angle = 0;
	car.tar_angle = angle;
	
	car.moter1.act_location = 0;
	car.moter1.act_speed = 0;
	car.moter1.dir = DIS;
	car.moter1.pwm_duty = 800;
	car.moter1.tar_location = 0;
	car.moter1.tar_speed = 0;
	
	car.moter2.act_location = 0;
	car.moter2.act_speed = 0;
	car.moter2.dir = DIS;
	car.moter2.pwm_duty = 800;
	car.moter2.tar_location = 0;
	car.moter2.tar_speed = 0;
	
	car.moter3.act_location = 0;
	car.moter3.act_speed = 0;
	car.moter3.dir = DIS;
	car.moter3.pwm_duty = 800;
	car.moter3.tar_location = 0;
	car.moter3.tar_speed = 0;
	
	car.moter4.act_location = 0;
	car.moter4.act_speed = 0;
	car.moter4.dir = DIS;
	car.moter4.pwm_duty = 800;
	car.moter4.tar_location = 0;
	car.moter4.tar_speed = 0;
	
}
/**************************************************************************
函数功能：配置基本定时器的中断优先级
输入变量：无
返回值	：无
说明    ：更改时需要更改中断源和优先级
**************************************************************************/
static void speed_BasicTim_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
  
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);                      // 设置中断组为0		
	
    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn ;	                     // 设置中断来源
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
int speed_count=0;
static void speed_BasicTim_Mode_Config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		
		
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);                  // 开启定时器时钟,即内部时钟CK_INT=72M
	
		TIM_TimeBaseStructure.TIM_Prescaler= 71;                              // 预分频器PSC的分频系数，让计数器计数一次的值为1us
    TIM_TimeBaseStructure.TIM_Period = 1*1000-1;	                          // 自动重装载寄存器的值，即设置计数器最大值，让计数次数为所设置的值加一
    
	  TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);                       // 初始化定时器
		
    TIM_ClearFlag(TIM7, TIM_FLAG_Update);                                 // 清除计数器中断标志位
    TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);                              // 计数器计数值达到最大后产生中断
		
    TIM_Cmd(TIM7, ENABLE);	                                              // 使能计数器
}

void speed_BasicTim_Init(void)
{
	speed_BasicTim_NVIC_Config();
	speed_BasicTim_Mode_Config();
}


