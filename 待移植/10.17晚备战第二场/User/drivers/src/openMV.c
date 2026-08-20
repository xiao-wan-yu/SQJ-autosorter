

/*-------------------------------文件包含--------------------------------*/
#include "stm32f10x.h"
#include "openMV.h"
#include "stm32f10x_rcc.h"
#include "usart.h"
/*-------------------------------全局变量--------------------------------*/

volatile char openMVData; //蓝牙数据
volatile char openMVData; //蓝牙数据
volatile char MVData[2]; //机械臂上的MV数据
volatile char MVData2[2]; //底盘上的MV数据



/*-------------------------------函数定义--------------------------------*/
/**************************************************************************
函数功能：初始化串口4的中断向量控制器
输入变量：无
返回值	：无
**************************************************************************/
void USART5_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStruct;
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStruct.NVIC_IRQChannel=UART5_IRQn;
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
void openMV_uart_init(void)
{
/*----------------------配置串口的GPIO口-----------------------*/
	GPIO_InitTypeDef GPIO_InitStructure;                                     //GPIO配置结构体

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);                    //打开GPIO时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;                                //配置Tx引脚PC10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                          //Tx的GPIO需要配置为推挽复用模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);                    //打开GPIO时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;                               //配置Rx引脚PC11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;                    // Rx的GPIO需要配置为浮空输入模式
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
/*------------------------配置串口寄存器-------------------------*/
	USART_InitTypeDef USART_InitStructure;                                   //定义串口配置结构体
		
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);                   // 打开串口外设的时钟，串口1在APB2，其他在APB1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);                   // 打开串口外设的时钟，串口2-5在APB1上
	
	
	USART_InitStructure.USART_BaudRate = 115200;                             // 配置波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;              // 配置 针数据字长
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                   // 配置停止位
	USART_InitStructure.USART_Parity = USART_Parity_No ;                     // 配置校验位
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;// 配置硬件流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;          // 配置工作模式，收发一起
	
	USART_Init(UART5, &USART_InitStructure);                                // 完成串口的初始化配置
/*------------------------使能中断、DMA等功能-------------------------*/
	
	                                                                         // 中断和DMA外设的配置需要先配置完，然后再开启串口的中断或DMA功能
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);	                         // 使能串口总线空闲中断
/*------------------------配置完成、进行使能-------------------------*/
	USART_Cmd(UART5, ENABLE);	                                             // 使能串口
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
void openMV_init(void)
{
	USART5_NVIC_Config();                                                    // 串口中断优先级配置
	openMV_uart_init();
	//openMVData=0;
	MVData[0] = ' ';
	MVData[0] = '\0';
}



/**************2025.9.6新加一个识别白线的串口************************/
/*-------------------------------函数定义--------------------------------*/
/**************************************************************************
函数功能：初始化串口2的中断向量控制器
输入变量：无
返回值	：无
**************************************************************************/
void USART2_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStruct;
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStruct.NVIC_IRQChannel=USART2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=1;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	
	NVIC_Init(&NVIC_InitStruct);
}

/**************************************************************************
函数功能：串口初始化
输入变量：无
返回值	：无
**************************************************************************/
void openMV_uart2_init(void)
{
/*----------------------配置串口的GPIO口-----------------------*/
	GPIO_InitTypeDef GPIO_InitStructure;                                     //GPIO配置结构体

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);                    //打开GPIO时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;                                //配置Tx引脚PC10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                          //Tx的GPIO需要配置为推挽复用模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);                    //打开GPIO时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;                               //配置Rx引脚PC11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;                    // Rx的GPIO需要配置为浮空输入模式
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
/*------------------------配置串口寄存器-------------------------*/
	USART_InitTypeDef USART_InitStructure;                                   //定义串口配置结构体
		
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);                   // 打开串口外设的时钟，串口1在APB2，其他在APB1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);                   // 打开串口外设的时钟，串口2-5在APB1上
	
	
	USART_InitStructure.USART_BaudRate = 115200;                             // 配置波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;              // 配置 针数据字长
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                   // 配置停止位
	USART_InitStructure.USART_Parity = USART_Parity_No ;                     // 配置校验位
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;// 配置硬件流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;          // 配置工作模式，收发一起
	
	USART_Init(USART2, &USART_InitStructure);                                // 完成串口的初始化配置
/*------------------------使能中断、DMA等功能-------------------------*/
	
	                                                                         // 中断和DMA外设的配置需要先配置完，然后再开启串口的中断或DMA功能
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);	                         // 使能串口总线空闲中断
/*------------------------配置完成、进行使能-------------------------*/
	USART_Cmd(USART2, ENABLE);	                                             // 使能串口
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
void openMV2_init(void)
{
	USART2_NVIC_Config();                                                    // 串口中断优先级配置
	openMV_uart2_init();
	MVData2[0] = 'a';
	MVData2[1] = '\0';
}


















