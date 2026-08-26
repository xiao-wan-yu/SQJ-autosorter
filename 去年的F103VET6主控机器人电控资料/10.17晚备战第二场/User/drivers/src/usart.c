#include "usart.h"
/**************************************************************************
函数功能：配置基本定时器的中断优先级
输入变量：无
返回值	：无
说明    ：更改时需要更改中断源和优先级
**************************************************************************/
static void USART1_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
  
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);                      // 设置中断组为0		
	
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn ;	                     // 设置中断来源
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	             // 设置主优先级为 0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	                   // 设置抢占优先级为3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
    NVIC_Init(&NVIC_InitStructure);
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
volatile uint32_t protocol_raw_data[128];                                            //陀螺仪原始数据
volatile uint8_t protocol_data[128];                                            //陀螺仪原始数据
void USART1_DMA_Config(void)
{
	DMA_InitTypeDef DMA_InitStruct;																						//定义初始化结构体
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);												//开启DMA1时钟
	
	DMA_DeInit(DMA1_Channel5);																								//复位DMA1——Channel6
	DMA_InitStruct.DMA_BufferSize=128;																          //传输的数据量，即传输次数
	DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralSRC;															//外设到存储器
	DMA_InitStruct.DMA_M2M=DMA_M2M_Disable;																		//关闭存储器到存储器模式
	DMA_InitStruct.DMA_MemoryBaseAddr=( uint32_t )protocol_raw_data;									  //存储器基地址为数组首地址，即数组名
	DMA_InitStruct.DMA_MemoryDataSize=DMA_MemoryDataSize_Word;								//存储器的数据长度类型是32位
	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;												//存储器地址增量模式
	DMA_InitStruct.DMA_Mode=DMA_Mode_Circular;																//不设置为循环模式
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)(&(USART1->DR));				  //外设的基地址
	DMA_InitStruct.DMA_PeripheralDataSize=DMA_MemoryDataSize_Word;				    //外设的数据长度类型是32位
	DMA_InitStruct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;								//外设地址不增加
	DMA_InitStruct.DMA_Priority=DMA_Priority_VeryHigh;												//DMA通道优先级为最高
	
	//DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);														//传输完成产生中断
	
	DMA_Init(DMA1_Channel5,&DMA_InitStruct);																	//初始化DMA1通道6 （未使能，即还没开启通道）
	DMA_Cmd(DMA1_Channel5 , ENABLE);                                          //开启DMA1通道1
}
/**************************************************************************

         配置串口一作为调试口，配置函数作为模板


函数功能：配置串口，串口初始化
输入变量：无
返回值	：无
说明　　：1、所有GPIO都挂载在APB2总线上，开启时钟的函数都是同一个
          2、USART1在APB2高速时钟上，其他在APB1上
          3、波特率可以设置为
											300
                      600
                      1200
                      2400
                      4800
                      9600
                      19200
                      38400
                      43000
                      56000
                      57600
                      115200
                      128000
											230400
											256000
											460800
				  4、一帧数据由起始位、数据位、校验位、停止位组成
						数据位：可以设置为8位或者9位，在串口正在接收或发送数据时不能设置M位
										M位是寄存器CR1中的字长位，用于设置串口数据位的长度是8还是9位。
						停止位：可以设置为0.5、1、1.5、2位，但UART4和5不能设置成0.5和1.5
						校验位：可以设置为奇校验、偶校验和无校验
					5、串口模式可以设置为接收模式、发送模式、或者接收发送一起
					6、硬件控制流模式用于同步通信，详细请查看手册25.3.14，非同步通信一般设置为none
					7、串口与中断和DMA之间的关系是协作关系，配置串口的时候可以选择是否使能这些功能
					   使能之前需要对协作的外设进行初始化配置，以免发生错误。
**************************************************************************/
void USART_Config(void)
{
/*----------------------配置串口的GPIO口-----------------------*/
	GPIO_InitTypeDef GPIO_InitStructure;                                     //GPIO配置结构体

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);                    //打开GPIO时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;                                //配置Tx引脚PA9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                          //Tx的GPIO需要配置为推挽复用模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;                               //配置Tx引脚PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;                    // Rx的GPIO需要配置为浮空输入模式
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
/*------------------------配置串口寄存器-------------------------*/
	USART_InitTypeDef USART_InitStructure;                                   //定义串口配置结构体
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);                   // 打开串口外设的时钟，串口1在APB2，其他在APB1
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);                   // 打开串口外设的时钟，串口2-5在APB1上
	
	
	USART_InitStructure.USART_BaudRate = 9600;                             // 配置波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;              // 配置 针数据字长
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                   // 配置停止位
	USART_InitStructure.USART_Parity = USART_Parity_No ;                     // 配置校验位
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;// 配置硬件流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;          // 配置工作模式，收发一起
	
	USART_Init(USART1, &USART_InitStructure);                                // 完成串口的初始化配置
/*------------------------使能中断、DMA等功能-------------------------*/
	USART1_NVIC_Config();                                                    // 串口中断优先级配置
//	USART1_DMA_Config();                                                     // 中断和DMA外设的配置需要先配置完，然后再开启串口的中断或DMA功能
	USART_ITConfig(USART1, /*USART_IT_IDLE*/USART_IT_RXNE, ENABLE);	                         // 不使能串口接收中断
//	USART_DMACmd(USART1, /*USART_DMAReq_Tx|*/USART_DMAReq_Rx, ENABLE);          // 不使能串口的发送和接收DMA功能
/*------------------------配置完成、进行使能-------------------------*/
	USART_Cmd(USART1, ENABLE);	                                             // 使能串口
}

/*****************  发送一个字节 **********************/
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
{
	/* 发送一个字节数据到USART */
	USART_SendData(pUSARTx,ch);
		
	/* 等待发送数据寄存器为空 */
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

/****************** 发送8位的数组 ************************/
void Usart_SendArray( USART_TypeDef * pUSARTx, uint8_t *array, uint16_t num)
{
  uint8_t i;
	
	for(i=0; i<num; i++)
  {
	    /* 发送一个字节数据到USART */
	    Usart_SendByte(pUSARTx,array[i]);	
  
  }
	/* 等待发送完成 */
	while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET);
}

/*****************  发送字符串 **********************/
void Usart_SendString( USART_TypeDef * pUSARTx, char *str)
{
	unsigned int k=0;
  do 
  {
      Usart_SendByte( pUSARTx, *(str + k) );
      k++;
  } while(*(str + k)!='\0');
  
  /* 等待发送完成 */
  while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET)
  {}
}

/*****************  发送一个16位数 **********************/
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch)
{
	uint8_t temp_h, temp_l;
	
	/* 取出高八位 */
	temp_h = (ch&0XFF00)>>8;
	/* 取出低八位 */
	temp_l = ch&0XFF;
	
	/* 发送高八位 */
	USART_SendData(pUSARTx,temp_h);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
	
	/* 发送低八位 */
	USART_SendData(pUSARTx,temp_l);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

///重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
		/* 发送一个字节数据到串口 */
		USART_SendData(DEBUG_USARTx, (uint8_t) ch);
		
		/* 等待发送完毕 */
		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}

///重定向c库函数scanf到串口，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
		/* 等待串口输入数据 */
		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(DEBUG_USARTx);
}


void USART4_Config(void)
{
/*----------------------配置串口的GPIO口-----------------------*/
	GPIO_InitTypeDef GPIO_InitStructure;                                     //GPIO配置结构体

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);                    //打开GPIO时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;                                //配置Tx引脚PA9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                          //Tx的GPIO需要配置为推挽复用模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
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
	//USART1_NVIC_Config();                                                    // 串口中断优先级配置
	//USART1_DMA_Config();                                                     // 中断和DMA外设的配置需要先配置完，然后再开启串口的中断或DMA功能
	//USART_ITConfig(USART1, USART_IT_IDLE/*USART_IT_RXNE*/, ENABLE);	                         // 不使能串口接收中断
	//USART_DMACmd(USART1, /*USART_DMAReq_Tx|*/USART_DMAReq_Rx, ENABLE);          // 不使能串口的发送和接收DMA功能
/*------------------------配置完成、进行使能-------------------------*/
	USART_Cmd(UART4, ENABLE);	                                             // 使能串口
}


