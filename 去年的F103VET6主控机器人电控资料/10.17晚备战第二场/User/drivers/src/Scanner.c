#include "Scanner.h"
#include "usart.h"

int QR_code_1=0,QR_code_2=0;//存放扫码结果，是一个整形数字
unsigned char Wakecmd[9] = {0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD};//唤醒扫码模块命令
/*---------------------------------函数定义----------------------------------*/
volatile char scannerData = 0;                                                      //





/****************************************************************************
函数功能：进行扫码
输入：无
输出：无
说明：扫码结果存放在全局变量QR_code_1和QR_code_2中，是一个整形数字
****************************************************************************/
#include "delay.h"
#include "Oled.h"
char Scan_QR()
{
	scannerData = 0;
	Usart_SendArray( USART1, Wakecmd, 9);
  //等待回收信号结束
		u8g2_ClearBuffer(&u8g2);
	u8g2_SetFont(&u8g2,u8g2_font_ncenB08_tf);
	oled_print_float( 10,30,(float)( 1));
	u8g2_SendBuffer(&u8g2);
	
  while (!scannerData);
	u8g2_ClearBuffer(&u8g2);
	u8g2_SetFont(&u8g2,u8g2_font_ncenB08_tf);
	oled_print_float( 10,30,(float)( 2));
	u8g2_SendBuffer(&u8g2);

	scannerData = 0;
	int i = 3000;
	while(i>0)
	{
		i--;
		if(scannerData)return scannerData;
//		if(scannerData[0]=='r')
//		{
//			return 'r';
//		}
//		else if(scannerData[0]=='b')
//		{
//			return 'b';
//		}
		delay_ms(1);
	}
	return 10;
}
