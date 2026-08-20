#include "task2.h"
#include "usart.h"                                                        /* 开发板硬件bsp头文件 */
#include "8LuHuiDu.h"



/**********************************************************************
  * @ 函数名  ： LED_Task
  * @ 功能说明： LED_Task任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/
void Task2_main(void)
{	
	uint8_t d=0;
    while (1)
    {
//			printf("2\n");
//			vTaskDelay(200);
			d=HuiDuOUT1<<7|HuiDuOUT2<<6|HuiDuOUT3<<5|HuiDuOUT4<<4|HuiDuOUT5<<3|HuiDuOUT6<<2|HuiDuOUT7<<1|HuiDuOUT8;
	    Usart_SendByte( USART1, (uint8_t)d);
			vTaskDelay(200);
    }
}























