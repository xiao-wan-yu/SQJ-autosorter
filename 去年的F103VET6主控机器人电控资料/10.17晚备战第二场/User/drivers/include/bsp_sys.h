#ifndef __BSP_SYS_H
#define __BSP_SYS_H 		

/*-------------------------------文件包含--------------------------------*/
#include "stm32f10x.h"

#define read_key_red GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3)
#define read_key_blue GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2)
#define read_MANFAN1 GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) //圆盘机斜射前方白线
#define read_MANFAN2 GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) //正前漫反射
#define read_MANFAN3 GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4) //灰度（在左后方）



//三个灰度 A4  E6 E5  灰度在机器人底下 
#define read_rhuidu GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4) //右灰度
#define read_mhuidu GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5) //中间灰度
#define read_lhuidu GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6) //左灰度
void key_Config(void);
void write_cri();

#endif





























