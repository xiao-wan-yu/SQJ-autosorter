#ifndef __SCANNER_H__
#define __SCANNER_H__

#include "stm32f10x.h"



extern int QR_code_1,QR_code_2;//存放扫码结果，是一个整形数字
extern unsigned char Wakecmd[9];

void Scaner_Init();
char Scan_QR();


#endif
