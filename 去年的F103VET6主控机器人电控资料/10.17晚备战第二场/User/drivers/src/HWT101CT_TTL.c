/*-------------------------------文件说明--------------------------------*/
/**************************************************************************
										该文件为HWT101CT_TTL陀螺仪驱动

产品特性：
串口通信波特率：4800-230400bps，默认为9600
回传速率：0.2-1000Hz，默认为10,目前200
工作电压5-36V，默认12V6.8mA
启动时间：1S
					重启       FF AA 00 FF 00 
					保存       FF AA 00 00 00 
					恢复出厂设置        01 00 
					设置波特率 FF AA 04 01 00   4800      bps
															02 00   9600
															03 00   19200
															04 00   38400
															05 00   57600
															06 00   115200
															07 00   230400
					设输出频率 FF AA 03 01 00   0.2       Hz
															02 00   0.5
															03 00   1
															04 00   2
															05 00   5
															06 00   10
															07 00   20
															08 00   50
															09 00   100
															0B 00   200
															0C 00   500
**************************************************************************/
/*-------------------------------文件包含--------------------------------*/
#include "stm32f10x.h"
#include "HWT101CT_TTL.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_dma.h"
#include "usart.h"

/*-------------------------------全局变量--------------------------------*/

volatile uint32_t raw_data[22];                                            //陀螺仪原始数据
volatile float angle;                                                      //陀螺仪实时角度
volatile float w_z;                                                      //陀螺仪实时角度加速度

/*-------------------------------函数定义--------------------------------*/
/**************************************************************************
函数功能：初始化串口1的中断向量控制器
输入变量：无
返回值	：无
**************************************************************************/
void USART3_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStruct;
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStruct.NVIC_IRQChannel=USART3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	
	NVIC_Init(&NVIC_InitStruct);
}
/**************************************************************************
函数功能：配置USART1的DMA
输入变量：无
返回值	：无
说明    ：1、不同的外设对应不同的DMA通道，USRAT2的RX对应DMA1通道6
          2、虽然串口的数据寄存器只有9位有效，并且通常只用低八位，
					   但是数据的大小还是需要设置成32位
					3、使用DMA进行数据传输时，如果数据量小，则可以传输完成后对数据进行读取使用
					   但是当传输的数据量比较大，并且两次传输的时间较短，则最好DMA传输一半时读取
						 然后再传输另一半再读取，避免出错。
					4、DMA与CPU对存储器的访问时交替的，内部有仲裁器，因此在DMA传输的时候可以直接程序读取数据，
					   但读取与DMA传输总有一个会挂起，因为他们对总线的控制是轮换的。
**************************************************************************/
void USART3_DMA_Config(void)
{
	DMA_InitTypeDef DMA_InitStruct;																						//定义初始化结构体
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);												//开启DMA1时钟
	
	DMA_DeInit(DMA1_Channel3);																								//复位DMA1——Channel6
	DMA_InitStruct.DMA_BufferSize=22;																          //传输的数据量，即传输次数
	DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralSRC;															//外设到存储器
	DMA_InitStruct.DMA_M2M=DMA_M2M_Disable;																		//关闭存储器到存储器模式
	DMA_InitStruct.DMA_MemoryBaseAddr=( uint32_t )raw_data;									  //存储器基地址为数组首地址，即数组名
	DMA_InitStruct.DMA_MemoryDataSize=DMA_MemoryDataSize_Word;								//存储器的数据长度类型是32位
	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;												//存储器地址增量模式
	DMA_InitStruct.DMA_Mode=DMA_Mode_Circular;																//不设置为循环模式
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)(&(USART3->DR));				  //外设的基地址
	DMA_InitStruct.DMA_PeripheralDataSize=DMA_MemoryDataSize_Word;				    //外设的数据长度类型是32位
	DMA_InitStruct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;								//外设地址不增加
	DMA_InitStruct.DMA_Priority=DMA_Priority_VeryHigh;												//DMA通道优先级为最高
	
	//DMA_ITConfig(DMA1_Channel6, DMA_IT_TC, ENABLE);														//传输完成产生中断
	
	DMA_Init(DMA1_Channel3,&DMA_InitStruct);																	//初始化DMA1通道6 （未使能，即还没开启通道）
	DMA_Cmd(DMA1_Channel3 , ENABLE);                                          //开启DMA1通道1
}
/**************************************************************************
函数功能：陀螺仪串口初始化
输入变量：无
返回值	：无
说明　　：使用串口2，TXPA2 RXPA3
**************************************************************************/
void HWT101CT_uart_init(void)
{
/*----------------------配置串口的GPIO口-----------------------*/
	GPIO_InitTypeDef GPIO_InitStructure;                                     //GPIO配置结构体

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);                    //打开GPIO时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;                                //配置Tx引脚PA9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                          //Tx的GPIO需要配置为推挽复用模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;                               //配置Tx引脚PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;                    // Rx的GPIO需要配置为浮空输入模式
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);               //开启复用功能寄存器时钟
	GPIO_PinRemapConfig(GPIO_FullRemap_USART3,ENABLE);                //
	
/*------------------------配置串口寄存器-------------------------*/
	USART_InitTypeDef USART_InitStructure;                                   //定义串口配置结构体
		
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);                   // 打开串口外设的时钟，串口1在APB2，其他在APB1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);                   // 打开串口外设的时钟，串口2-5在APB1上
	
	
	USART_InitStructure.USART_BaudRate = 9600;                             // 配置波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;              // 配置 针数据字长
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                   // 配置停止位
	USART_InitStructure.USART_Parity = USART_Parity_No ;                     // 配置校验位
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;// 配置硬件流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;          // 配置工作模式，收发一起
	
	USART_Init(USART3, &USART_InitStructure);                                // 完成串口的初始化配置
/*------------------------使能中断、DMA等功能-------------------------*/
	
	                                                                         // 中断和DMA外设的配置需要先配置完，然后再开启串口的中断或DMA功能
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);	                         // 使能串口总线空闲中断
	
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);                          // 使能串口的接收DMA功能
/*------------------------配置完成、进行使能-------------------------*/
	USART_Cmd(USART3, ENABLE);	                                             // 使能串口
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
#include "Oled.h"
void HWT101_Z(void)
{
  uint8_t buff1[5] = {0XFF, 0XAA, 0X69, 0X88, 0XB5};//解锁陀螺仪配置指令
  uint8_t buff2[5] = {0XFF, 0XAA, 0X76, 0X00, 0X00};//Z轴归零
	uint8_t buff3[5] = {0XFF, 0XAA, 0X00, 0XFF, 0X00};//重启

	Usart_SendArray( USART3, &buff1[0], 5);     //解锁陀螺仪配置指令
	delay_ms(100);
	Usart_SendArray( USART3, &buff2[0], 5);     //Z轴归零
	delay_ms(100);
	Usart_SendArray( USART3, &buff3[0], 5);     //Z轴归零
	delay_ms(100);
}
void HWT101CT_init(void)
{

	USART3_NVIC_Config();                                                    // 串口中断优先级配置
	USART3_DMA_Config();
	HWT101CT_uart_init();                                                    //初始化串口
	
	HWT101_Z();
	delay_ms(300);
//	angle = 0;while(!angle);
}

///**************************************************************************
//函数功能：配置USART2的中断
//输入变量：无
//返回值	：无
//说明    ：1、根据手册，IDLE的中断标志位的清除需要软件序列：
//             先读寄存器SR再度寄存器DR，即先后调用函数
//					   USART_GetITStatus(USART2, USART_IT_IDLE);
//		         USART_ReceiveData(USART2);
//					2、判断是否传输正确，如果出错
//					   则说明通信受到干扰，此时DMA传输会一直错位，因此需要重新设置
//						 DMA传输，让其重新接收新的一帧数据
//**************************************************************************/
//#include "HWT101CT_TTL.h"
//#include "usart.h"
//void USART2_IRQHandler(void)
//{
//	if(USART_GetITStatus(USART2, USART_IT_IDLE)){
//		USART_ReceiveData(USART2);                                              //清除中断标志位
//		if(((0xA8+raw_data[15]+raw_data[16]+raw_data[17]+raw_data[18])&0xFF)\
//			==raw_data[21])                                                       //帧尾校验判断传输是否正确
//		{
//			angle = ((float)((raw_data[18]<<8)|raw_data[17]))/32768*180;          //计算并更新角度值
//			Usart_SendByte( USART1, (uint8_t)angle);
//		}
//		else
//		{
//			DMA_Cmd(DMA1_Channel6 , DISABLE);                                     //关闭DMA1通道6
//			DMA1_Channel6->CNDTR=22; 																						  //重新设置传输数量
//			DMA_Cmd(DMA1_Channel6 , ENABLE);                                      //开启DMA1通道6
//		}
//	}
//}

