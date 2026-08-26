#ifndef __OPENMV_H
#define ___OPENMV_H

/*-------------------------------文件包含--------------------------------*/
#include "stm32f10x.h"

extern volatile char openMVData;                                             //蓝牙数据
extern volatile char MVData[2]; //蓝牙数据
extern volatile char MVData2[2]; //蓝牙数据


void openMV_init(void);
void openMV2_init(void);
#endif





























