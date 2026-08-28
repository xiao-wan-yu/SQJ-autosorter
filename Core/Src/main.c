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
#include "./../../Mycode/hwt101ct.h"
#include "./../../Mycode/robot.h"
#include "./../../Mycode/vision.h"

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
  while (1)
  {
    GRAY_Update();
    //OLED_Printf(0, 0,  OLED_8X16_HALF, "G1:%1d%1d%1d%1d%1d%1d%1d%1d",
    //            GRAY_Data[GRAY1][0],GRAY_Data[GRAY1][1],GRAY_Data[GRAY1][2],GRAY_Data[GRAY1][3],
    //            GRAY_Data[GRAY1][4],GRAY_Data[GRAY1][5],GRAY_Data[GRAY1][6],GRAY_Data[GRAY1][7]);
    // OLED_Printf(0, 32, OLED_8X16_HALF, "G3:%1d%1d%1d%1d%1d%1d%1d%1d",
    //             GRAY_Data[GRAY3][0],GRAY_Data[GRAY3][1],GRAY_Data[GRAY3][2],GRAY_Data[GRAY3][3],
    //             GRAY_Data[GRAY3][4],GRAY_Data[GRAY3][5],GRAY_Data[GRAY3][6],GRAY_Data[GRAY3][7]);
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

      /*
        执行扫球动作
      */

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

        if(1)//结束倒球信号
        {
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

            /*先校准左右再校准前后，前后都需要盲走了*/
            //
            
          
          }
        }

      }

      UART1_Data[0]=0;
    }
    
    //单独测试转圈：围绕外径8cm柱子做圆周运动（测距仅定半径，绕圈全程不读测距 → 不会因测距丢目标狂动）
    if(UART1_Data[0]==5)
    {

      UART1_Data[0]=0;                            // 立即清指令，防止循环重复触发
      UART1_Printf("circle start\r\n");

      /* ===== 测距定参考距离：多次采样只收有效值(目标区间10~20cm)，取中值抗杂散 =====
         前提：车头已正对柱子（GY53_2 读到的是 传感器→柱面 的距离） */
      uint16_t d_ok[10];                          // 有效采样缓存
      uint8_t  n = 0;                             // 有效采样个数
      for(uint8_t i = 0; i < 12; i++){            // 最多采12次
        uint16_t dd = GY53_GetDistance_PWM(GY53_2_GPIO_Port, GY53_2_Pin);
        if(dd >= 80 && dd <= 220){                // 只收8~22cm：丢目标返回大值/杂散直接丢弃
          d_ok[n++] = dd;
          if(n >= 10) break;
        }
        HAL_Delay(30);                            // 采样间隔，避开电机/震动噪声
      }
      UART1_Printf("valid=%d\r\n", n);
      if(n < 3){                                  // 有效采样太少：保护退出，绝不乱转
        UART1_Printf("no pipe!\r\n");
      }
      else
      {
        /* 冒泡排序取中值：比平均更抗单次大值/小值 */
        for(uint8_t i = 0; i < n-1; i++)
          for(uint8_t j = i+1; j < n; j++)
            if(d_ok[j] < d_ok[i]){ uint16_t t = d_ok[i]; d_ok[i] = d_ok[j]; d_ok[j] = t; }
        uint16_t d_ref = d_ok[n/2];               // 目标测距 mm
        UART1_Printf("ref=%dmm\r\n", d_ref);

        const float GY53_2_OFFSET_CM = 15.0f;     // 传感器到车中心纵向距离(cm)（实测15.0cm）
        const float PIPE_RADIUS_CM   = 4.0f;      // 柱子外径8cm → 半径4cm

        /* ===== 闭环绕柱 v4（测距径向闭环 + 陀螺仪航向前馈，完全不用里程计位置） =====
           反馈(径向):测距 → v_y = KP×(测距-目标) 纠偏保半径（真正的测距闭环）
           前馈(航向):w = v_t/(测距+偏置+柱半径) → 车头随圈转、始终指向圆心
           绕圈进度 = 陀螺仪累积转角；测距无效时保持原速绕行等恢复，
           持续无效超时 → 停车，绝不盲目漂移 */
        flag.angle = 0;                           // 角度环让位，w 由本闭环接管
        chassis.v_x = 0.0f;  chassis.v_y = 0.0f;  chassis.w = 0.0f;
        chassis.x_speed_plan_flag = 0;
        chassis.y_speed_plan_flag = 0;
        chassis.x_set_speed_flag  = 1;            // 手动设速，防控制循环归零
        chassis.y_set_speed_flag  = 1;

        float d_ref_cm = (float)d_ref/10.0f;      // 目标测距 cm
        float d_filt   = d_ref_cm;                // 滤波测距 cm
        float yaw_last = HWT101CT_Data.yaw;
        float yaw_acc  = 0.0f;                    // 已绕角度(°)
        uint32_t t_prt = HAL_GetTick();
        uint8_t  lost      = 0;                   // 连续无效计数
        uint8_t  lost_stop = 0;                   // 丢目标停车标志
        float next_align = 90.0f;                 // 下一次摆头校准的绕圈进度(°)：每90°校准一次
        
        /*调参调这四个
      //切向速度：沿圆周切线方向走多快
      大：绕圈快但 GY53 采样相对变"稀"、更容易丢目标,`w` 更大,整体更激进
      小：绕圈慢、更稳,测距采样更密、不易丢目标;但一圈时间变长(8→6 约从 26s 变 35s)
      v_t    = 8.0f 

      //径向纠偏增益：负责把车拉回目标半径。
      大：纠偏更"猛",偏离后快速拉回;但容易超调来回振荡,圈不圆
      小：纠偏柔和、不易振荡;但收敛变慢,抗扰动弱,太小会"追不上"半径
      KP_R   = 1.0f

      //径向限幅：无论误差多大,径向修正速度不超过 ±VY_MAX
      大：大误差时能快速大幅径向修正;但车会猛斜插进圈,轨迹波动大
      小：修正动作柔和、轨迹顺滑;但大误差时拉回慢(若 `KP_R` 又大,会被限幅"卡住"发挥不出来)
      VY_MAX = 3.0f

      //测距低通系数："新测量"和"历史平滑值"的权重
      大：跟踪快,真实距离变化立即反映;但 GY53 的跳变噪声也进来,`e`/`v_y`/`w` 会抖,圈不圆
      小：非常平滑、抗噪声;但响应滞后,车对真实变化"慢半拍",可能追不上半径
      D_FILT = 0.15f     
      
        */
        const float v_t    = 8.0f;                // 切向速度 cm/s（>0逆时针 / <0顺时针）
        const float KP_R   = 1.0f;                // 测距径向纠偏增益（已从3.0调到1.0：更柔和防来回猛拉）
        const float VY_MAX = 3.0f;                // 径向速度限幅 cm/s（从5.0降到3.0：防止猛斜插）
        const float D_FILT  = 0.15f;               // 测距低通滤波系数（从0.3降到0.15：更平滑防噪声）

        while(fabsf(yaw_acc) < 355.0f){           // 绕满一整圈
          uint16_t dd = GY53_GetDistance_PWM(GY53_2_GPIO_Port, GY53_2_Pin);
          if(dd >= 60 && dd <= 260){              // 有效读数（放宽范围）
            d_filt += D_FILT * ((float)dd/10.0f - d_filt);  // 有效帧必更新（普通低通压噪）
            lost = 0;
          }else{                                  // 无效/丢目标
            if(++lost >= 100){ lost_stop = 1; break; }   // 持续约3.5s无效才停车
          }

          float e     = d_filt - d_ref_cm;        // 测距径向误差（闭环反馈量）
          float r_est = d_filt + GY53_2_OFFSET_CM + PIPE_RADIUS_CM;
          if(r_est < 10.0f) r_est = 10.0f;

          chassis.v_x = v_t;                      // 切向恒定
          chassis.v_y = KP_R * e;                 // 径向：远了前进(朝圆心)、近了后退
          if(chassis.v_y >  VY_MAX) chassis.v_y =  VY_MAX;
          else if(chassis.v_y < -VY_MAX) chassis.v_y = -VY_MAX;
          chassis.w = v_t / r_est;                // 航向前馈：车头随圈转，始终指向圆心
          if(chassis.w >  YAW_PID_OUT_MAX) chassis.w =  YAW_PID_OUT_MAX;
          else if(chassis.w < -YAW_PID_OUT_MAX) chassis.w = -YAW_PID_OUT_MAX;

          /* 打印(每200ms)：d=滤波测距 e=径向误差 vx/vy=切向/径向 w=角速度 */
          if(HAL_GetTick() - t_prt >= 200){
            UART1_Printf("d=%d e=%d vx=%d vy=%d w=%d l=%d\r\n",
                         (int)(d_filt*10.0f), (int)(e*10.0f),
                         (int)chassis.v_x, (int)chassis.v_y,
                         (int)(chassis.w*100.0f), lost);
            t_prt = HAL_GetTick();
          }

          /* 陀螺仪累积转角判断已绕角度 */
          float ddg = HWT101CT_Data.yaw - yaw_last;
          yaw_last = HWT101CT_Data.yaw;
          if(ddg > 180.0f)       ddg -= 360.0f;
          else if(ddg < -180.0f) ddg += 360.0f;
          yaw_acc += ddg;

          /* ===== 摆头校准：每绕90°，停车原地左右摆头，找测距最小值方向(=正对水管中间) =====
             车头正对水管时 GY53 光束垂直打柱面 → 测距最小；偏角越大读数越大。
             校准后车头回正对水管，d_filt 重置为对准后的真实距离，继续绕圈。 */
          if(fabsf(yaw_acc) >= next_align){
            next_align += 90.0f;
            chassis.v_x = 0.0f; chassis.v_y = 0.0f; chassis.w = 0.0f;  // 停车
            HAL_Delay(100);

            float yaw_start = HWT101CT_Data.yaw;
            float yaw_l     = yaw_start;
            float yaw_min   = yaw_start;
            uint16_t d_min  = 0xFFFF;
            float sw        = 0.0f;                // 摆动累积角(°)
            const float W_SW   = 0.4f;             // 摆动角速度 rad/s（约23°/s）
            const float SWING  = 12.0f;            // 摆动幅度(°)

            /* ① 向右摆 SWING°（顺时针，yaw 增），期间采样找最小测距 */
            chassis.w = -W_SW;
            while(sw < SWING){
              uint16_t sdd = GY53_GetDistance_PWM(GY53_2_GPIO_Port, GY53_2_Pin);
              if(sdd >= 40 && sdd <= 300 && sdd < d_min){ d_min = sdd; yaw_min = HWT101CT_Data.yaw; }
              float dy = HWT101CT_Data.yaw - yaw_l;
              yaw_l = HWT101CT_Data.yaw;
              if(dy > 180.0f) dy -= 360.0f; else if(dy < -180.0f) dy += 360.0f;
              sw += dy;
              HAL_Delay(10);
            }
            /* ② 向左摆回并多摆 SWING°（逆时针，yaw 减，sw 从 +SWING 到 -SWING） */
            chassis.w = W_SW;
            while(sw > -SWING){
              uint16_t sdd = GY53_GetDistance_PWM(GY53_2_GPIO_Port, GY53_2_Pin);
              if(sdd >= 40 && sdd <= 300 && sdd < d_min){ d_min = sdd; yaw_min = HWT101CT_Data.yaw; }
              float dy = HWT101CT_Data.yaw - yaw_l;
              yaw_l = HWT101CT_Data.yaw;
              if(dy > 180.0f) dy -= 360.0f; else if(dy < -180.0f) dy += 360.0f;
              sw += dy;
              HAL_Delay(10);
            }
            /* ③ 转回最小测距方向（正对水管中间） */
            chassis.w = 0.0f;
            float err = yaw_min - HWT101CT_Data.yaw;
            if(err > 180.0f) err -= 360.0f; else if(err < -180.0f) err += 360.0f;
            while(fabsf(err) > 1.0f){
              chassis.w = (err > 0.0f) ? -0.3f : 0.3f;
              float dy = HWT101CT_Data.yaw - yaw_l;
              yaw_l = HWT101CT_Data.yaw;
              if(dy > 180.0f) dy -= 360.0f; else if(dy < -180.0f) dy += 360.0f;
              err -= dy;
              HAL_Delay(10);
            }
            chassis.w = 0.0f;
            HAL_Delay(50);
            if(d_min != 0xFFFF){
              d_filt = (float)d_min/10.0f;         // 重置为对准后的真实测距
              UART1_Printf("align dmin=%d\r\n", d_min);
            }else{
              UART1_Printf("align FAIL\r\n");
            }
            yaw_last = HWT101CT_Data.yaw;          // 校准后重新累积绕圈进度
            t_prt    = HAL_GetTick();              // 校准期间不打印
          }

          HAL_Delay(10);                          // 控制周期10ms
        }
        /* 停车 + 恢复角度环（重新锁向当前朝向） */
        chassis.v_x = 0.0f;  chassis.v_y = 0.0f;  chassis.w = 0.0f;
        chassis.x_set_speed_flag = 0;
        chassis.y_set_speed_flag = 0;
        flag.angle = 1;
        chassis.target_yaw = YAW_TARGET_NONE;
        if(lost_stop) UART1_Printf("LOST! stop\r\n");
        else          UART1_Printf("circle done\r\n");
      }
    }

    if(KEY_ONE(KEY0_GPIO_Port, KEY0_Pin)){
      ROBOT_Move(-50, 390, 100, 100, 100, 100);

      ROBOT_Angle(270);
    }

    if(KEY_ONE(KEY1_GPIO_Port, KEY1_Pin)){
      ROBOT_Move(-50, 0, 10, 10, 10, 10);

      //ROBOT_Angle(270);
    }

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
