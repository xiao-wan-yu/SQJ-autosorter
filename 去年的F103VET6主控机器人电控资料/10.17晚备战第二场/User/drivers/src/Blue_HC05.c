
/**************************************************************************
蓝牙模块使用方法：
利用上位机配置好后，蓝牙模块与单片机的通信就只是简单的串口通信，即当两个蓝牙模块是一根连接的数据线即可，
单片机相互之间的通信就是简单的串口通信

蓝牙模块的配置：利用电脑串口调试助手
一个HC-05配置为主机：AT模式波特率38400
+ADDR:98d3:31:fdc551

另一个HC-05配置为主机：AT模式波特率38400
+ADDR:98d3:32:20d623
**************************************************************************/
/**************************************************************************
HC-05
工作模式：
上电默认工作模式：数据传输模式，LED快闪
AT模式：EN引脚高电平进入，低电平退出，AT模式下LED慢闪，或者长按按键上电进入AT模式
蓝牙配对成功：LED快闪两次停顿一次

参数：
主从一体，默认从机
波特率：4800-1382400，AT模式默认38400，数据透传模式默认9600
电压：3.6-6V
蓝牙地址：+ADDR:98d3:31:fdc551

引脚：
STATE：输出引脚，配对成功输出高电平，未配对输出低电平
RX：3.3VTTL
TX：3.3VTTL
GND：
VCC：3.6-6V
EN：下拉输入引脚，高电平进入AT模式，低电平退出AT模式

AT指令：HEX，命令后的回车换行为0x0D 0x0A，调试助手输入需要打回车，不能用\r\n
指令: 41 54 0D 0A (AT)                                   返回: 4F 4B 0D 0A (OK)  意义：开始AT指令通信
      41 54 2b 52 4f 4c 45 3d 30 0D 0A (AT+ROLE=0)             4F 4B 0D 0A (OK)  意义：设置为从机模式
      41 54 2b 52 4f 4c 45 3d 30 0D 0A (AT+ROLE=1)             4F 4B 0D 0A (OK)  意义：设置为主机模式
			41 54 2b 50 53 57 44 3d 31 32 33 34 0D 0A (AT+PSWD=1234) 4F 4B 0D 0A (OK)  意义：设置配对码为1234，其中0x30为字符0
			41 54 2b 41 44 44 52 3f 0D 0A (AT+ADDR?)    
      41 54 2b 42 54 4e 44 3d 0D 0A (AT+BTNT=) 	
			41 54 2b 43 4d 4f 44 45 3d 30 0D 0A (AT+CMODE=0)         4F 4B 0D 0A (OK)  意义：设置为绑定模式，指定蓝牙地址连接
			41 54 2b 43 4d 4f 44 45 3d 31 0D 0A (AT+CMODE=1)         4F 4B 0D 0A (OK)  意义：设置为任意蓝牙地址连接模式
			41 54 2b 52 45 53 45 54 0D 0A (AT+RESET)                 4F 4B 0D 0A (OK)  意义：重启，即退出 AT模式，回到透传模式
			41 54 2b 4f 52 47 4c 0D 0A (AT+ORGL)                     4F 4B 0D 0A (OK)  意义：恢复出厂设置
**************************************************************************/
/*-------------------------------文件包含--------------------------------*/
#include "stm32f10x.h"
#include "Blue_HC05.h"
#include "stm32f10x_rcc.h"
#include "usart.h"
/*-------------------------------全局变量--------------------------------*/

volatile float BlueData;                                                      //蓝牙数据


/*-------------------------------函数定义--------------------------------*/
/**************************************************************************
函数功能：初始化串口4的中断向量控制器
输入变量：无
返回值	：无
**************************************************************************/
void USART4_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStruct;
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStruct.NVIC_IRQChannel=UART4_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=1;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	
	NVIC_Init(&NVIC_InitStruct);
}

/**************************************************************************
函数功能：蓝牙串口初始化
输入变量：无
返回值	：无
说明　　：使用串口4，TXPC10 RXPC11
**************************************************************************/
void Blue_uart_init(void)
{
/*----------------------配置串口的GPIO口-----------------------*/
	GPIO_InitTypeDef GPIO_InitStructure;                                     //GPIO配置结构体

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);                    //打开GPIO时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;                                //配置Tx引脚PC10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                          //Tx的GPIO需要配置为推挽复用模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;                               //配置Rx引脚PC11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;                    // Rx的GPIO需要配置为浮空输入模式
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
/*------------------------配置串口寄存器-------------------------*/
	USART_InitTypeDef USART_InitStructure;                                   //定义串口配置结构体
		
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);                   // 打开串口外设的时钟，串口1在APB2，其他在APB1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);                   // 打开串口外设的时钟，串口2-5在APB1上
	
	
	USART_InitStructure.USART_BaudRate = 9600;                             // 配置波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;              // 配置 针数据字长
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                   // 配置停止位
	USART_InitStructure.USART_Parity = USART_Parity_No ;                     // 配置校验位
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;// 配置硬件流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;          // 配置工作模式，收发一起
	
	USART_Init(UART4, &USART_InitStructure);                                // 完成串口的初始化配置
/*------------------------使能中断、DMA等功能-------------------------*/
	
	                                                                         // 中断和DMA外设的配置需要先配置完，然后再开启串口的中断或DMA功能
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);	                         // 使能串口总线空闲中断
/*------------------------配置完成、进行使能-------------------------*/
	USART_Cmd(UART4, ENABLE);	                                             // 使能串口
}
/**************************************************************************
函数功能：初始化陀螺仪
输入变量：无
返回值	：无
说明    ：1、陀螺仪初始化完成后，USART和DMA以及中断就已经开始工作，
             这些外设会自动的将传感器的数据传输到raw_data数组中，
						 并且raw_data会以10Hz的频率刷新
					2、利用串口2读取数据，然后用DMA传输到内存中，传输完成后判断
					   数据是否有效，如果有效则计算并更新角度，angle变量的值刷新，
						 如果数据无效，说明传输出问题，需重置DMA
**************************************************************************/
void Blue_init(void)
{
	USART4_NVIC_Config();                                                    // 串口中断优先级配置
	Blue_uart_init();
	BlueData=0;
}




















