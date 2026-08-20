
/*-------------------------------文件包含--------------------------------*/

#include "Oled.h"

/*-------------------------------函数定义--------------------------------*/
/**************************************************************************
函数功能：初始化oled的GPIO
输入变量：无
返回值	：无
**************************************************************************/
void IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	
	   
	GPIO_InitStructure.GPIO_Pin = SCL_Pin|SDA_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(IIC_GPIO_Port, &GPIO_InitStructure);
}



	



/**************************************************************************
函数功能：显示u8g2库的logo
输入变量：&u8g2:
返回值	：无
**************************************************************************/
void draw(u8g2_t *u8g2)
{
	u8g2_ClearBuffer(u8g2); 
	
    u8g2_SetFontMode(u8g2, 1);                                              /*字体模式选择*/
    u8g2_SetFontDirection(u8g2, 0);                                         /*字体方向选择*/
    u8g2_SetFont(u8g2, u8g2_font_inb24_mf);                                 /*字库选择*/
    u8g2_DrawStr(u8g2, 0, 20, "U");
    
    u8g2_SetFontDirection(u8g2, 1);
    u8g2_SetFont(u8g2, u8g2_font_inb30_mn);
    u8g2_DrawStr(u8g2, 21,8,"8");
        
    u8g2_SetFontDirection(u8g2, 0);
    u8g2_SetFont(u8g2, u8g2_font_inb24_mf);
    u8g2_DrawStr(u8g2, 51,30,"g");
    u8g2_DrawStr(u8g2, 67,30,"\xb2");
    
    u8g2_DrawHLine(u8g2, 2, 35, 47);
    u8g2_DrawHLine(u8g2, 3, 36, 47);
    u8g2_DrawVLine(u8g2, 45, 32, 12);
    u8g2_DrawVLine(u8g2, 46, 33, 12);
  
    u8g2_SetFont(u8g2, u8g2_font_4x6_tr);
    u8g2_DrawStr(u8g2, 1,54,"github.com/olikraus/u8g2");
	
	u8g2_SendBuffer(u8g2);
	delay_ms(300);
}

/**************************************************************************
函数功能：
输入变量：
返回值	：无
**************************************************************************/
static uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    switch (msg)
    {
    case U8X8_MSG_DELAY_100NANO: // delay arg_int * 100 nano seconds
        __NOP();
        break;
    case U8X8_MSG_DELAY_10MICRO: // delay arg_int * 10 micro seconds
        for (uint16_t n = 0; n < 320; n++)
        {
            __NOP();
        }
        break;
    case U8X8_MSG_DELAY_MILLI: // delay arg_int * 1 milli second
        delay_ms(1);
        break;
    case U8X8_MSG_DELAY_I2C: // arg_int is the I2C speed in 100KHz, e.g. 4 = 400 KHz
        delay_us(5);
        break;                    // arg_int=1: delay by 5us, arg_int = 4: delay by 1.25us
    case U8X8_MSG_GPIO_I2C_CLOCK: // arg_int=0: Output low at I2C clock pin
		if(arg_int == 1) 
		{
			GPIO_SetBits(IIC_GPIO_Port, SCL_Pin);
		}
		else if(arg_int == 0)
		{
			GPIO_ResetBits(IIC_GPIO_Port, SCL_Pin);  
		}  
        break;                    // arg_int=1: Input dir with pullup high for I2C clock pin
    case U8X8_MSG_GPIO_I2C_DATA:  // arg_int=0: Output low at I2C data pin
        if(arg_int == 1) 
		{
			GPIO_SetBits(IIC_GPIO_Port, SDA_Pin);
		}
		else if(arg_int == 0)
		{
			GPIO_ResetBits(IIC_GPIO_Port, SDA_Pin);  
		} 
        break;                    // arg_int=1: Input dir with pullup high for I2C data pin
    case U8X8_MSG_GPIO_MENU_SELECT:
        u8x8_SetGPIOResult(u8x8, /* get menu select pin state */ 0);
        break;
    case U8X8_MSG_GPIO_MENU_NEXT:
        u8x8_SetGPIOResult(u8x8, /* get menu next pin state */ 0);
        break;
    case U8X8_MSG_GPIO_MENU_PREV:
        u8x8_SetGPIOResult(u8x8, /* get menu prev pin state */ 0);
        break;
    case U8X8_MSG_GPIO_MENU_HOME:
        u8x8_SetGPIOResult(u8x8, /* get menu home pin state */ 0);
        break;
    default:
        u8x8_SetGPIOResult(u8x8, 1); // default return value
        break;
    }
    return 1;
}
/**************************************************************************
函数功能：初始化u8g2
输入变量：&u8g2:
返回值	：无
**************************************************************************/
void u8g2Init(u8g2_t *u8g2)
{
	u8g2_Setup_ssd1306_i2c_128x64_noname_f(u8g2, U8G2_R0, u8x8_byte_sw_i2c,\
	u8x8_gpio_and_delay);                                                     // 初始化 u8g2 结构体
	u8g2_InitDisplay(u8g2);                                                   // 根据所选的芯片进行初始化工作，初始化完成后，显示器处于关闭状态
	u8g2_SetPowerSave(u8g2, 0);                                               // 打开显示器
	u8g2_ClearBuffer(u8g2);
}
/**************************************************************************
函数功能：初始化Oled
输入变量：无
返回值	：无
**************************************************************************/
void oled_print_float(u8g2_uint_t x,u8g2_uint_t y,float value)
{
    //u8g2_ClearBuffer(&u8g2);                                              //清除缓冲区
			
		char buff[20];                                                          //定义字符串用于存放进度条百分比
		sprintf(buff,"%.2f",value);                                             //将"%d",(int)(i/80.0*100)格式化到buff中
		

		u8g2_SetFont(&u8g2,u8g2_font_ncenB08_tf);
		u8g2_DrawStr(&u8g2,x,y,buff);                                         
		

}
/**************************************************************************
函数功能：初始化Oled
输入变量：无
返回值	：无
**************************************************************************/
u8g2_t u8g2;
void Oled_Init(void)
{
	IIC_Init();
	u8g2Init(&u8g2);
	draw(&u8g2);
}
