/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "oled_ui/oled.h"
#include "oled_ui/oled_driver.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include <stm32f4xx_hal.h>
#include "./../../Mycode/led.h"
#include "./../../Mycode/oled_api.h"
#include "./../../Mycode/uart.h"
#include "./../../Mycode/delay.h"
#include "./../../Mycode/key.h"
#include "./../../Mycode/buzzer.h"
#include "./../../Mycode/laser.h"
#include "./../../Mycode/gy53.h"
#include "./../../Mycode/tb6612.h"
#include "./../../Mycode/chassis.h"
#include "./../../Mycode/encoder.h"
#include "./../../Mycode/serialplot.h"
#include "./../../Mycode/pid.h"
#include "./../../Mycode/myflash.h"
#include "./../../Mycode/storage.h"
#include "./../../Mycode/gw_grayscale.h"
#include "./../../Mycode/tcs34725.h"
#include "./../../Mycode/hwt101ct.h"
#include "./../../Mycode/robot.h"
#include "./../../Mycode/vision.h"
#include "./../../Mycode/lobot_servo.h"
#include "./../../Mycode/circle.h"

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include "stm32f407xx.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_tim_ex.h"
#include "stm32f4xx_hal_uart.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* FLAG 结构体定义已移至 Mycode/chassis.h（main.c 定义变量、chassis.c 等模块引用） */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* UART1 接收模板：帧格式 S;A;B;C;D;E;F;G（8个32位整数，分号分隔） */
#define UART1_DATA_NUM 8                     // 每帧数据个数，按需修改
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
FLAG flag;

int16_t data_encoder = 0;

int32_t UART1_Data[UART1_DATA_NUM] = {0};    // 存放解析后的8个32位整数



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

  if(htim == &htim7){ //1ms产生一次中断

    // static uint8_t count1 = 0;
    // if(++count1 >= 5){
    //   count1 = 0;
    //   ICM42688Mahony_Update();
    // }

    // static uint16_t ms_cnt = 0;
    // if(++ms_cnt >= 10){          // 每10ms执行一次车体控制周期
    //   ms_cnt = 0;
    //   CAR_Control_Loop();        // PID速度闭环 + 麦轮运动学 + 里程计
    // }


    //hwt101ct陀螺仪数据更新
    static uint8_t count1 = 0;
    if(flag.hwt101ct && ++count1 >= 5){
      count1 = 0;
      if(HWT101CT_RxFlag){
        HWT101CT_RxFlag = 0;
        HWT101CT_Update();
      }
    }


    //车体10ms控制周期：速度闭环（第一阶段）
    static uint16_t count2 = 0;
    if(flag.chassis && ++count2 >= 10){
      count2 = 0;
      CHASSIS_Control_Loop();      // 4轮速度环（PID + 编码器 + PWM）
    }

  }
  /* 清零法无需编码器溢出中断（TIM3/TIM4），故无 htim3/htim4 分支 */

}





/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_UART5_Init();
  MX_TIM7_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */


  OLED_Init();


  /*外设启动区域*/
  HAL_TIM_Base_Start_IT(&htim7);//开启1ms中断

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);//开启车轮的PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);//开启车轮的encoder
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_2);
  /* 编码器采用清零法：每10ms读CNT后清零，计数器不会溢出，无需开启溢出中断（之前开中断但缺处理函数会导致卡死） */
  HAL_GPIO_WritePin(TB6612_STBY_GPIO_Port, TB6612_STBY_Pin, GPIO_PIN_SET);//TB6612使能

  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART1_RxBuf, UART1_RxLength);//调试串口
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);    //使用DMA+UART时，会开启传输过半中断，需手动关闭
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UART2_RxBuf, UART2_RxLength);//主视觉串口
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);    //使用DMA+UART时，会开启传输过半中断，需手动关闭
  // HAL_UARTEx_ReceiveToIdle_DMA(&huart5, UART5_RxBuf, UART5_RxLength);//步进电机串口
  // __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);    //使用DMA+UART5时，会开启传输过半中断，需手动关闭

  HAL_Delay(300);

  HWT101CT_Init();//陀螺仪初始化
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART3_RxBuf, UART3_RxLength);//陀螺仪串口
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);    //使用DMA+UART时，会开启传输过半中断，需手动关闭
  flag.hwt101ct = 1;

  CHASSIS_Init();//底盘初始化：配置4轮速度环PID参数 + 航向环(角度环)参数
  flag.chassis = 1;
  flag.angle   = 1;   // 航向环（角度环）默认开启：上电锁定当前朝向，串口 tyaw 可遥控转向


  // /*主视觉测试（临时注释：先验证灰度，测完恢复）*/
  // while(1){
  //   if(VISION1_RxFlag){
  //     VISION1_RxFlag = 0;
  //     VISION_ReceiveData(VISION1_RxBuf, VISION1_RxRealLength);
  //     OLED_Printf(0, 0, OLED_8X16_HALF, "suc:%1d peri:%1d", VISION_Data.success, VISION_Data.period);
  //     OLED_Printf(0, 16, OLED_8X16_HALF, "tar:%1d", VISION_Data.target);
  //     OLED_Printf(0, 32, OLED_8X16_HALF, "x:%3d y:%3d", VISION_Data.x, VISION_Data.y);
  //     OLED_Printf(0, 48, OLED_8X16_HALF, "dis%4d", VISION_Data.distance);
  //     OLED_Update();
  //   }
  // }


  // while(1){
  //   if(KEY_ONE(KEY0_GPIO_Port, KEY0_Pin)){
  //     ROBOT_Move(-50, 390, 100, 100, 100, 100);


  //     ROBOT_Angle(270);
  //   }

  //   /* 串口打印角度环数据（目标/实际/输出w + 里程计位置x/y），SerialPlot 观察走直线纠偏/转向收敛
  //      并解析串口调参指令：kp/ki/kd/target 角度环、vx/vy 手动、mx/my 走距、mv/mvacc 规划速度 */
  //   UART1_Printf("%f %f %f %f %f\r\n",
  //                chassis.target_yaw,
  //                HWT101CT_Data.yaw,
  //                chassis.yaw_pid.out,
  //                chassis.pos_x,
  //                chassis.pos_y);
  //   if(UART1_RxFlag){
  //     UART1_RxFlag = 0;
  //     SERIALPLOT_ChangeParam((char *)UART1_RxBuf);
  //   }
  //   HAL_Delay(10);
  // }


  /*速度环调参测试（临时注释：先跑下方编码器裸测标定 ACCURACY，测完恢复）*/
  // while(1){
  //   OLED_Printf(0, 0, OLED_8X16_HALF, "kp:%06.2f", chassis.speed_pid[1].kp);
  //   OLED_Printf(0, 16, OLED_8X16_HALF, "ki:%06.2f", chassis.speed_pid[1].ki);
  //   OLED_Printf(0, 32, OLED_8X16_HALF, "kd:%06.2f", chassis.speed_pid[1].kd);
  //   OLED_Printf(0, 48, OLED_8X16_HALF, "tar:%06.2f", chassis.speed_pid[1].target);
  //   OLED_Update();
  //   UART1_Printf("%f %f %f %f\r\n", chassis.speed_pid[1].target, chassis.speed_pid[1].actual, chassis.speed_pid[1].out, chassis.speed_pid[1].errorint);
  //   if(UART1_RxFlag){
  //     UART1_RxFlag = 0;
  //     SERIALPLOT_ChangeParam((char *)UART1_RxBuf);
  //   }
  // }
  // /*激光传感器测试--通过*/
  // while(1){
  //   OLED_Printf(0, 0, OLED_8X16, "barrier:%1d", LASER_Barrier(LASER1_GPIO_Port, LASER1_Pin));
  //   OLED_Update();
  // }
  // /*测距传感器--通过*/
  // while(1){
  //   OLED_Printf(0, 0, OLED_8X16, "distance:%4d", GY53_GetDistance_PWM(GY53_1_GPIO_Port, GY53_1_Pin));
  //   OLED_Update();
  // }
  // /*灰度传感器--通过*/
  /* 灰度传感器读取：GRAY1/GRAY3 都走串行 IO 接口（GPIO 模拟时钟），读 8 路数字量（0=深、1=浅）
     引脚（CubeMX 配置）：GRAY1=PB4(DAT)/PB9(CLK)、GRAY3=PB6(DAT)/PB7(CLK)
     OLED 第一行(y=0)显示 GRAY1 八通道，第三行(y=32)显示 GRAY3 八通道 */
  
  // /*陀螺仪测试--通过*/
  // HWT101CT_Init();
  // HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART3_RxBuf, UART3_RxLength);//陀螺仪串口
  // __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);    //使用DMA+UART时，会开启传输过半中断，需手动关闭
  // flag.hwt101ct = 1;
  // while(1){
  //   OLED_Printf(0, 0, OLED_8X16_HALF, "yaw:%6.2f", HWT101CT_Data.yaw);
  //   OLED_Update();
  // }
  // /*电机PWM、encoder测试--通过*/
  // int i = 0;
  // while(1){
  //   if(KEY_ONE(KEY3_GPIO_Port, KEY3_Pin)){
  //     i++;
  //     if(i > 10) i = -10;
  //     TB6612_Control(MOTOR_Left_Front, i*100);
  //     TB6612_Control(MOTOR_Left_Back, i*100);
  //     TB6612_Control(MOTOR_Right_Back, i*100);
  //     TB6612_Control(MOTOR_Right_Front, i*100);
  //     OLED_Printf(0, 0, OLED_8X16_HALF, "PWM: %+4d", i * 100);
  //     OLED_Update();
  //   }
  //   OLED_Printf(0, 16, OLED_8X16_HALF, "L_F:%+3d L_B:%+3d", ENCODER_GetPulse(ENCODER_LeftFront), ENCODER_GetPulse(ENCODER_LeftBack));
  //   OLED_Update();
  //   HAL_Delay(10);
  // }
  // /* 临时调参：SerialPlot 串口画图 + 在线改PID（调好即删，改回正常逻辑） */
  // SERIALPLOT_PIDAdjustParam();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  

  //   HAL_Delay(2000);
  //   if (GW_Gray_Init() == 0) {
  //   UART1_Printf("GW Gray online\r\n");
  // } else {
  //   UART1_Printf("GW Gray offline\r\n");
  // }
  // ROBOT_MoveSpeed(0,30);
  // HAL_Delay(2000);
  // ROBOT_MoveSpeed(0,0);
  // ROBOT_Move(0, 200, 30, 30, 30, 30);

  /* ==================== TCS34725 颜色识别（替换原 1 号灰度传感器 GRAY1） ====================
     SCL=PB9、SDA=PB4（原 GRAY1 的 CLK/DAT 线位），软件 I2C 100kHz，无需改 CubeMX；
     模块供电 3.3V，LED/INT 悬空。GRAY3 仍走串行接口用于循线（GRAY_Data[GRAY3] 依旧可用）。
     判色由 TCS34725_ClassifyColor() 完成：只分 黑/非黑 两类（红蓝统称非黑），
     串口打印原始 C/R/G/B + HSV，可接 SerialPlot 观察并按实际物体标定阈值（见 tcs34725.h）。 */
  uint8_t tcs_online = TCS34725_Init();
  UART1_Printf("TCS34725 %s, ID=0x%02X\r\n",
               tcs_online ? "ONLINE" : "OFFLINE", TCS34725_GetID());

  while (1)
  {
    /* GRAY3 仍走串行更新（循线数据 GRAY_Data[GRAY3] 保持有效） */
    GRAY3_Serial_Update();

    /* TCS34725 读颜色：OLED 显示颜色名 + R/G/B + 亮度（串口不自动刷新，只在按键采样时打印） */
    static TCS34725_RGBC tcs_rgbc = {0};
    if(TCS34725_GetRawData(&tcs_rgbc)){
      uint8_t tcs_col = TCS34725_ClassifyColor(&tcs_rgbc);
      OLED_Printf(0, 0,  OLED_8X16_HALF, "col:%s", TCS34725_ColorName(tcs_col));
      OLED_Printf(0, 16, OLED_8X16_HALF, "R:%3d G:%3d B:%3d", tcs_rgbc.r, tcs_rgbc.g, tcs_rgbc.b);
      OLED_Printf(0, 48, OLED_8X16_HALF, "C:%4d V:%3d%%", tcs_rgbc.c, (uint8_t)(tcs_rgbc.v * 100));
      delay_ms(200);
    } else {
      OLED_Printf(0, 0, OLED_8X16_HALF, "TCS OFFLINE");
    }

    /* ==================== 颜色采样：按一次按钮 = 串口发一条当前值 ====================
       把黑/红/蓝物体放到传感器下，按对应按钮一次，串口立即打一条 H/S/V：
         按钮2(KEY1)=黑 -> BLK H=.. S=.. V=..
         按钮3(KEY2)=红 -> RED H=.. S=.. V=..
         按钮4(KEY3)=蓝 -> BLU H=.. S=.. V=..
       每色按 8 次共 24 条，直接复制发回来定阈值。 */
    if(KEY_ONE(KEY1_GPIO_Port, KEY1_Pin)){                 /* 按钮2：黑 */
      TCS34725_GetRawData(&tcs_rgbc);                     /* 按下瞬间重新采一次 */
      UART1_Printf("BLK H=%5.1f S=%0.2f V=%0.2f\r\n", tcs_rgbc.h, tcs_rgbc.s, tcs_rgbc.v);
    }
    if(KEY_ONE(KEY2_GPIO_Port, KEY2_Pin)){                 /* 按钮3：红 */
      TCS34725_GetRawData(&tcs_rgbc);
      UART1_Printf("RED H=%5.1f S=%0.2f V=%0.2f\r\n", tcs_rgbc.h, tcs_rgbc.s, tcs_rgbc.v);
    }
    if(KEY_ONE(KEY3_GPIO_Port, KEY3_Pin)){                 /* 按钮4：蓝 */
      TCS34725_GetRawData(&tcs_rgbc);
      UART1_Printf("BLU H=%5.1f S=%0.2f V=%0.2f\r\n", tcs_rgbc.h, tcs_rgbc.s, tcs_rgbc.v);
    }

    HAL_Delay(50);
    //测距，2是前面的，1是后面的
    //OLED_Printf(0,16 , OLED_8X16_HALF, "dis_1:%4d", GY53_GetDistance_PWM(GY53_1_GPIO_Port, GY53_1_Pin));
    OLED_Printf(0,32 , OLED_8X16_HALF, "dis_2:%4d", GY53_GetDistance_PWM(GY53_2_GPIO_Port, GY53_2_Pin));
    //激光
    //OLED_Printf(0, 48 ,OLED_8X16_HALF, "ba_3:%1d", LASER_Barrier(LASER3_GPIO_Port, LASER3_Pin));
    //OLED_Printf(64, 48, OLED_8X16_HALF, "ba_2:%1d", LASER_Barrier(LASER2_GPIO_Port, LASER2_Pin));
    // OLED_Printf(32, 32, OLED_8X16_HALF, "bar_3:%1d", LASER_Barrier(LASER3_GPIO_Port, LASER3_Pin));
    OLED_Update();


    /* ===== UART1 数据接收模板：收到 "S,A,B,C,D,E,F,G" 一帧后解析到 UART1_Data[0..7] ===== */
    if(UART1_RxFlag){                                // DMA空闲中断收到一帧后置1
      UART1_RxFlag = 0;                              // 必须立即清零
      char line[UART1_RxLength + 1];                 // 拷贝一份并补'\0'（DMA缓冲末尾没有结束符）
      uint16_t len = UART1_RxRealLength;
      if(len > UART1_RxLength) len = UART1_RxLength; // 防越界
      memcpy(line, UART1_RxBuf, len);
      line[len] = '\0';
      uint8_t i = 0;
      char *p = strtok(line, ",");                   // 按分号切段
      while(p && i < UART1_DATA_NUM){                // 逐段转成32位整数，支持负数
        UART1_Data[i++] = (int32_t)strtol(p, NULL, 10);
        p = strtok(NULL, ",");
      }
      /* 回显验证（测试用，实际使用可删） */
      UART1_Printf("S=%d A=%d B=%d C=%d D=%d E=%d F=%d G=%d\r\n",
                   UART1_Data[0], UART1_Data[1], UART1_Data[2], UART1_Data[3],
                   UART1_Data[4], UART1_Data[5], UART1_Data[6], UART1_Data[7]);
    }

    //1;-50;390;100;100;100;100;270
    if(UART1_Data[0]==3){
    ROBOT_Move(UART1_Data[1], UART1_Data[2], UART1_Data[3], UART1_Data[4], UART1_Data[5], UART1_Data[6]);
    ROBOT_Angle(UART1_Data[7]);
    UART1_Data[0]=0;
    }
    
    //完整走
    if(UART1_Data[0]==4)
    {
      /**************圆盘机****************/
    
      //这里是机械臂抬起，该动作组运行时间为1秒
			//delay_ms(1300);
      
      //先盲走到圆盘机中心+面向
      ROBOT_Move(-60,408,100,120,100,120);
      HAL_Delay(100);
      UART1_Printf("1");
      ROBOT_Angle(270);
      UART1_Printf("2");

      
      //向前慢走，直到灰度传感器边缘的两个传感器有感应到白线
      GRAY_Update();
      ROBOT_MoveSpeed(0, 15);
      while(GRAY_Data[GRAY3][0]!=1)
      {
        GRAY_Update();
      }
      ROBOT_MoveSpeed(0, 0);
      // ROBOT_Move(0, -5, 0, 20, 0, 20);

      //这里是机械臂拍球动作
		  //delay_ms(1400);
		  //Usart_SendByte(UART5, 0xA1);//发0xA1告诉视觉开始识别

      /*移植过来的视觉处理代码
      
	MVData[0]=0x00;
	uint8_t first_flag = 0;
	
	while(MVData[0] != 0x36){
		
		if(MVData[0] == 0x34) {//红
			if(!first_flag){
				
				first_flag++;
				MVData[0]=0x00;
			}
			else {
				
				runActionGroup(77, 1);
				delay_ms(350);
				MVData[0]=0x00;
			}
			
		}
			
		if(MVData[0] == 0x35) {//黄
			if(!first_flag){
				
				first_flag++;
				MVData[0]=0x00;
			}
			else {
				
				runActionGroup(99, 1);
				delay_ms(350);
				MVData[0]=0x00;
			}
			
		}
	}
	
	
	while(MVData[0] != 0x36);    //mv端任务完成
      */


      /***************去仓库倒球*************/

      if(1)//结束扫球信号
      {
        //退后固定距离
        ROBOT_Move(0,-25,50,50,50,50);
        //向左平行到仓库
        ROBOT_Move(-185,0,100,0,100,0);
        //转身
        HAL_Delay(100);
        ROBOT_Angle(90);
        
        //往后慢退，直到测距测得合适距离（适合倒球的距离）
        UART1_Printf("3");
        ROBOT_MoveSpeed(0, -10);
        while(GY53_GetDistance_PWM(GY53_1_GPIO_Port, GY53_1_Pin)>100);
        ROBOT_MoveSpeed(0,0);
        
        //定位操作：向左慢平移到左后光电感应到无障碍物，之后再往右走固定距离（刚到对上仓库的距离）
        UART1_Printf("4");
        ROBOT_MoveSpeed(-20, 0);
        while (LASER_Barrier(LASER1_GPIO_Port, LASER1_Pin)==1);
        ROBOT_MoveSpeed(0,0);

        ROBOT_Move(20, 0, 20, 0, 50, 0);

        //runActionGroup(51, 1); 	//这里是倒球动作组
	      //delay_ms(2000);

        if(1)//结束倒球信号
        {
          
          //runActionGroup(50, 1); 	//这里是收倒球槽
	        ///delay_ms(2000);
          
          UART1_Printf("5");
          //右+前，移动到阶梯附近
          ROBOT_Move(80, 160, 100, 100, 100, 100);
          // ROBOT_Move(0,160,0,100,0,100);
          UART1_Printf("6");
          //向左慢走，直到前面的两个光电都感应到障碍物，开始测距，然后向前走到合适的距离（适合识别的距离）
          ROBOT_MoveSpeed(-10, 0);
          while(LASER_Barrier(LASER2_GPIO_Port, LASER2_Pin)==0);
          ROBOT_MoveSpeed(0, 10);
          UART1_Printf("7");
          while(GY53_GetDistance_PWM(GY53_2_GPIO_Port, GY53_2_Pin)>100);
          ROBOT_MoveSpeed(0, 0);

          //向左走到字母处
          ROBOT_Move(-30, 0, 50, 0, 50, 0);
          /*
            这里放识别的代码
          */
          ROBOT_MoveSpeed(-20, 0);
          HAL_Delay(1000);//避免路上识别到字母然后停下来
          while(LASER_Barrier(LASER3_GPIO_Port,LASER3_Pin)==1);//这里容易识别到字母上的黑，然后停下来
          ROBOT_MoveSpeed(0, 0);
          UART1_Printf("8");

          //左前光电无障碍物就开始从左往右走
          ROBOT_MoveSpeed(20, 0);
          /*
            这里放抓取的代码
          */
          HAL_Delay(3000);
          while(LASER_Barrier(LASER2_GPIO_Port,LASER2_Pin)==1);//这里容易识别到字母上的黑，然后停下来
          ROBOT_MoveSpeed(0, 0);
          //右前光电无障碍物，就开始向左走固定距离（走到阶梯平面中间）
          ROBOT_Move(-45, -45, 50, 50, 50, 50);
          if(1)//开始执行圆柱任务的信号
          {
            ROBOT_Angle(270);
            //这里是转圈函数

            /*
            这里放识别的代码，如果遇到可以夹的就停下转圈
            */

            //转完一圈，收起机械臂，然后往左转身走到仓库中间倒方块
            ROBOT_Move(-60, 0, 100, 100, 100, 100);
            ROBOT_Angle(90);//车子前面朝右
            ROBOT_Move(0, -125, 50, 50, 50, 50);
            ROBOT_Move(-45, 0, 50, 50, 50, 50);
            UART1_Printf("9");

            //定位操作：向左慢平移到左后光电感应到无障碍物，之后再往右走固定距离（刚到仓库中间的距离）
            ROBOT_MoveSpeed(-20, 0);
            while (LASER_Barrier(LASER1_GPIO_Port, LASER1_Pin)==1);
            ROBOT_MoveSpeed(0,0);

            ROBOT_Move(25, 0, 20, 0, 50, 0);

            //得走远一点才能转身倒方块
            ROBOT_Move(0, 10, 50, 50, 50, 50);
            ROBOT_Angle(270);//车子前面朝左
            UART1_Printf("10");
            /*
              这里放倒方块的代码
            */

            //倒完方块转个身再回家
            ROBOT_Angle(0);
            ROBOT_Move(60, -210, 50, 100, 50, 100);
            UART1_Printf("11");

            /*先校准左右再校准前后，左右走可能会抖，而且前后比左右的反馈更准
            注意！！！必须先让颜色传感器在左右移动之后一定能进入红色区域，
            即前后距离必须能确保在红色区域（在哪里无所谓，后面再校准）
            */
            //如果为黑色，往右走，此时左右距离没问题，且传感器在红色区域内
            ROBOT_Move(10, 0, 50, 50, 50, 50);
            //往前走，走到颜色传感器一定在黑色区域内
            ROBOT_Move(0, 50, 50, 50, 50, 50);
            //颜色传感器校准前后：如果为黑色，往后走，直到为红色
            ROBOT_Move(0, 50, 50, 50, 50, 50);
            //识别为红色后停下
          }
        }

      }

      UART1_Data[0]=0;
    }
    
    //单独测试转圈：定半径圆周运动（豆包三层闭环方案：径向距离环PID + 航向同步环 + 切向速度前馈）
    //  目标测距 d_target_mm（传感器→管壁）：150~200mm 范围内都可用；轨迹半径 D=d/10+传感器偏置8+管半径4=27~32cm
    //  在线调参：串口指令 cstage i N（N=0~4 分步调试：0纯开环→1加测距→2加距离环→3加航向环→4加漂移修正）
    //            ckp/cki/ckd/cvy/cyawkp/calpha/cstep/cdriftper/cprint 含义见 Mycode/circle_params.h
    //  前提：调用前车头已正对水管（GY53_2 读到的是 传感器→管壁 的距离，不是斜距）
    if(UART1_Data[0]==5)
    {
      UART1_Data[0]=0;                            // 立即清指令，防止循环重复触发
      UART1_Printf("circle start\r\n");

      /* 目标测距 175mm（15~20cm 范围内任取）、公转角速度 0.35rad/s（切向速度≈0.35×30≈10.5cm/s）、
         绕满整圈、逆时针。CIRCLE_Run 内部：先采测距定初始半径 → 按当前 cstage 分级闭环绕圈 →
         绕满弧角自动停；期间每 cprint(默认200)ms 串口打印一次状态，SerialPlot 直接看。 */
      CIRCLE_Run(GY53_2_GPIO_Port, GY53_2_Pin, 175, 0.35f, 360, 1);

      UART1_Printf("circle done\r\n");
    }

    if(UART1_Data[0]==6)
    {
      UART1_Data[0]=0;
      /*先校准左右再校准前后，左右走可能会抖，而且前后比左右的反馈更准
      注意！！！必须先让颜色传感器在左右移动之后一定能进入红/蓝区域，
      即前后距离必须能确保在红/蓝区域内（在哪里无所谓，后面再校准）
      */
      //如果为黑色，匀速往右走，直到传感器进入红/蓝区域
      //去抖：连续3次(约150ms)都读到红/蓝才确认，交界处"红黑红黑"抖动不会误停
      ROBOT_MoveSpeed(10, 0);
      {
        uint8_t stable = 0;
        while(1){
          TCS34725_GetRawData(&tcs_rgbc);
          if(TCS34725_ClassifyColor(&tcs_rgbc) != TCS_COLOR_BLACK){
            if(++stable >= 3) break;
          } else {
            stable = 0;
          }
          HAL_Delay(50);
        }
      }
      
      //因为加了消抖，所以会稍微多走一小点，再减少一点盲走的距离

      //进入红/蓝后，继续向右多走12，确保停在红/蓝区域内部（避免停在边缘抖动；距离按区域宽度调整），而且确保车身左右都在红/蓝区域内
      ROBOT_Move(12, 0, 10, 10, 100, 100);
      
      //往前走，走到颜色传感器一定在黑色区域内（同样连续3次确认）
      ROBOT_MoveSpeed(0, 10);
      {
        uint8_t stable = 0;
        while(1){
          TCS34725_GetRawData(&tcs_rgbc);
          if(TCS34725_ClassifyColor(&tcs_rgbc) == TCS_COLOR_BLACK){
            if(++stable >= 3) break;
          } else {
            stable = 0;
          }
          HAL_Delay(50);
        }
      }
      
      //颜色传感器校准前后：如果为黑色，匀速往后走，直到进入红/蓝区域（连续3次确认）
      ROBOT_MoveSpeed(0, -10);
      {
        uint8_t stable = 0;
        while(1){
          TCS34725_GetRawData(&tcs_rgbc);
          if(TCS34725_ClassifyColor(&tcs_rgbc) != TCS_COLOR_BLACK){
            if(++stable >= 3) break;
          } else {
            stable = 0;
          }
          HAL_Delay(50);
        }
      }
      
      //识别为红/蓝后继续向后多走3，确保停在红/蓝区域内部（避免停在边缘抖动；距离按区域宽度调整），而且确保车身前后都在红/蓝区域内
      ROBOT_Move(0, -3, 10, 10, 100, 100);
      ROBOT_MoveSpeed(0, 0);
    }

    if(KEY_ONE(KEY0_GPIO_Port, KEY0_Pin)){
      ROBOT_Move(-50, 390, 100, 100, 100, 100);

      ROBOT_Angle(270);
    }

    //if(KEY_ONE(KEY1_GPIO_Port, KEY1_Pin)){
    //  //ROBOT_Move(-50, 0, 10, 10, 10, 10);
    //  runActionGroup(50, 1);

      //ROBOT_Angle(270);
    //}

    // /* 串口打印角度环数据（目标/实际/输出w + 里程计位置x/y），SerialPlot 观察走直线纠偏/转向收敛
    //    并解析串口调参指令：kp/ki/kd/target 角度环、vx/vy 手动、mx/my 走距、mv/mvacc 规划速度 */
    // UART1_Printf("%f %f %f %f %f\r\n",
    //              chassis.target_yaw,
    //              HWT101CT_Data.yaw,
    //              chassis.yaw_pid.out,
    //              chassis.pos_x,
    //              chassis.pos_y);
    // if(UART1_RxFlag){
    //   UART1_RxFlag = 0;
    //   SERIALPLOT_ChangeParam((char *)UART1_RxBuf);
    // }
   
    // UART1_Printf("GY1:%d  GY2:%d\r\n",GY53_GetDistance_PWM(GY53_1_GPIO_Port, GY53_1_Pin),GY53_GetDistance_PWM(GY53_2_GPIO_Port, GY53_2_Pin));
    // OLED_Printf(0, 0, OLED_8X16_HALF, "ni%d %d",GY53_GetDistance_PWM(GY53_1_GPIO_Port, GY53_1_Pin),GY53_GetDistance_PWM(GY53_2_GPIO_Port, GY53_2_Pin));
    // OLED_Update();
    // HAL_Delay(10);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
