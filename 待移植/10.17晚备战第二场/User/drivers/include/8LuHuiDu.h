#ifndef __8LUHUIDU_H
#define __8LUHUIDU_H	

/*-------------------------------ÎÄ¼þ°üº¬--------------------------------*/

#define HuiDuOUT1 GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_13)
#define HuiDuOUT2 GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_14)
#define HuiDuOUT3 GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11)
#define HuiDuOUT4 GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_12)
#define HuiDuOUT5 GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_9)
#define HuiDuOUT6 GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_10)
#define HuiDuOUT7 GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7)
#define HuiDuOUT8 GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_8)

#define HongWai   GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_14)

void HuiDu_GPIO_Config(void);


#endif





























