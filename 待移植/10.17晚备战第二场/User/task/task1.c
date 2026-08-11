#include "task1.h"
#include "8LuHuiDu.h"
#include "usart.h"   
#include "openMV.h"
#include "task_timer.h"


/**********************************************************************
  * @ 函数名  ： LED_Task
  * @ 功能说明： LED_Task任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/

void Task1_main(void)
{	

    while (1)
    {
			//printf("1\n");
			//vTaskDelay(200);
			Usart_SendByte( UART5, 1);
			//while(openMVData==0);
			//while(HongWai   ==0);										//等接收货物
			Task_TIM_NewState(ENABLE);
			vTaskSuspend(NULL);
	
    }
}























