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
#include "i2c.h"
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
#include "./../../Mycode/mpu6050.h"
#include "./../../Mycode/gray.h"
#include "./../../Mycode/buzzer.h"
#include "./../../Mycode/laser.h"
#include "./../../Mycode/infrared.h"
#include "./../../Mycode/gy53.h"
#include "./../../Mycode/ultrasonic.h"
#include "./../../Mycode/servo.h"
#include "./../../Mycode/tb6612.h"
#include "./../../Mycode/encoder.h"
#include "./../../Mycode/emm_v5.h"
#include "./../../Mycode/serialplot.h"
#include "./../../Mycode/pid.h"
#include "./../../Mycode/icm42688.h"
#include "./../../Mycode/myflash.h"
#include "./../../Mycode/storage.h"
#include "./../../Mycode/nmos.h"
#include "./../../Mycode/motor/motor.h"

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include "oled.h"
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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_TEST_PWM   200   // 四轮同转测试: 1/4满转=25%占空比 (MY_PWM_MAX=800 的25%)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

  if(htim == &htim7){ //1ms产生一次中断--主要应用于PID计时

    // static uint8_t count1 = 0;
    // if(++count1 >= 5){
    //   count1 = 0;
    //   ICM42688Mahony_Update();
    // }

    //===== 诊断LED1: 每1s翻转一次，证明 TIM7 中断活着 =====
    static uint16_t led1_cnt = 0;
    if(++led1_cnt >= 1000){
      led1_cnt = 0;
      HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    }
    //====================================================

    //===== 电机10ms控制环: 每10次(10ms)调用一次 time_period_fun() =====
    static uint8_t motor_pid_cnt = 0;
    if(++motor_pid_cnt >= 10){
      motor_pid_cnt = 0;
      time_period_fun();

      //===== 诊断LED2: 控制环在跑且 PWM 非0 则点亮，证明在驱动电机 =====
      if(my_car.motor_1.PWM != 0 || my_car.motor_2.PWM != 0 ||
         my_car.motor_3.PWM != 0 || my_car.motor_4.PWM != 0)
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET); // LED2亮(低电平有效)
      else
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);   // LED2灭
      //=============================================================
    }
    //=================================================================

  }

}

//w25q64测试--失败 可能是模块本身有问题
// void W25Q64_Save(uint8_t byte){
//   //写使能
//   uint8_t WriteEnableCmd[] = {0x06};
//   HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
//   HAL_SPI_Transmit(&hspi3, WriteEnableCmd, 1, HAL_MAX_DELAY);
//   HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
//   //扇区擦除
//   uint8_t EraseSectorCmd[] = {0x20, 0x00, 0x00, 0x00};
//   HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
//   HAL_SPI_Transmit(&hspi3, EraseSectorCmd, 4, HAL_MAX_DELAY);
//   HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
//   delay_ms(100);
//   //写使能
//   HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
//   HAL_SPI_Transmit(&hspi3, WriteEnableCmd, 1, HAL_MAX_DELAY);
//   HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
//   //页编程
//   uint8_t PageProgramCmd[] = {0x02, 0x00, 0x00, 0x00, byte};
//   HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
//   HAL_SPI_Transmit(&hspi3, PageProgramCmd, 5, HAL_MAX_DELAY);
//   HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
//   delay_ms(10);
// }
// uint8_t W25Q64_Read(void){
//   uint8_t Read_DataCmd[] = {0x03, 0x00, 0x00, 0x00};
//   uint8_t data = 0xff;
//   HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
//   HAL_SPI_Transmit(&hspi3, Read_DataCmd, 4, HAL_MAX_DELAY);
//   HAL_SPI_Receive(&hspi3, &data, 1, HAL_MAX_DELAY); 
//   HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
//   return data;
// }



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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_UART5_Init();
  MX_TIM7_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  // uint8_t a = 'n';

  /* 电机子系统初始化: TIM1四路PWM + TB6612方向引脚 + 4路编码器 + car_init */
  motor_init();

  /* ============ 电机四轮同转开环测试（1/4满转=25%占空比，临时，诊断完删除） ============
   * 强制 motor_dir=+1, 让 PWM 符号直接对应 TB6612 的 IN 状态:
   *   PWM=+200 (MY_PWM_MAX=800 的 25%) -> IN1=0, IN2=1, 正转
   * 四个轮子同时一直正转, 便于观察:
   *   四轮都转(同方向)        -> 电机驱动/接线正常, 剩下只是极性标定问题
   *   某轮不转/蜂鸣           -> 该通道 IN1/IN2 那一路有问题(硬件/接线)
   * 观察 e 值 (OLED 第6行 e1~e4):
   *   轮子转但 e 一直是0       -> 该轮编码器没计数(编码器接线)
   *   e 有值但速度乱跳         -> 编码器方向/极性标定问题
   * ================================================================ */
  HAL_TIM_Base_Start_IT(&htim7);   // 关键：启动1ms中断 -> 控制环每10ms运行一次
  w_set_flag = 1;                  // 关闭角度环
  motor_openloop = 1;              // 开环：跳过PID，直接按 PWM 驱动
  my_car.motor_1.motor_dir = 1;    // 测试时强制，让 PWM 符号 = IN 状态
  my_car.motor_2.motor_dir = 1;
  my_car.motor_3.motor_dir = 1;
  my_car.motor_4.motor_dir = 1;
  my_car.motor_1.PWM = MOTOR_TEST_PWM;   // 四轮同时一直正转 25% 占空比
  my_car.motor_2.PWM = MOTOR_TEST_PWM;
  my_car.motor_3.PWM = MOTOR_TEST_PWM;
  my_car.motor_4.PWM = MOTOR_TEST_PWM;
  my_car.v_x = 0.0f;
  my_car.w   = 0.0f;
  my_car.v_y = 0.0f;
  /* ====================================================================== */


  /*注意：更改了serialplot文件中的函数和内容，但是由于没来得及测试，所以暂时不知道有没有bug*/
  #if 0
  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, STEP_RxBuf, STEP_RxLength);
  HAL_Delay(500);   //等待系统稳定
  Emm_V5_Origin_Set_O(1, 1);  //设置当前位置为零点
  HAL_Delay(10);
  Emm_V5_Origin_Set_O(2, 1);  //设置当前位置为零点
  HAL_Delay(10);
  Emm_V5_Synchronous_motion(0);
  HAL_Delay(10);
  HAL_Delay(500);

  Emm_V5_Pos_Control(1, 0, 2000, 0, 1600, 0, 1);  //设置位置
  HAL_Delay(10);
  Emm_V5_Pos_Control(2, 0, 2000, 0, 1600, 0, 1);  //设置位置
  HAL_Delay(10);
  Emm_V5_Synchronous_motion(0);
  HAL_Delay(10);
  HAL_Delay(1000);

  Emm_V5_Vel_Control(1, 0, 2000, 200, 1); //设置为速度模式
  HAL_Delay(10);
  Emm_V5_Synchronous_motion(0);
  HAL_Delay(10);
  HAL_Delay(500);
  
  Emm_V5_Vel_Control(1, 0, 2000, 100, 1); //设置为速度模式
  HAL_Delay(10);
  Emm_V5_Vel_Control(2, 0, 1000, 200, 1); //设置为速度模式
  HAL_Delay(10);
  Emm_V5_Synchronous_motion(0);
  HAL_Delay(10);
  HAL_Delay(500);

  Emm_V5_Stop_Now(1, 1);  //设置为立即停止
  HAL_Delay(10);
  Emm_V5_Stop_Now(2, 1);  //设置为立即停止
  HAL_Delay(10);
  Emm_V5_Synchronous_motion(0);
  HAL_Delay(500);
  #endif

  uint32_t lastTick = HAL_GetTick();
  uint32_t oledTick  = HAL_GetTick();

  /* 烧录成功测试：程序启动后 UART4(PC10-TX) 会先发一次 666 */
  UART4_Printf("666\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  OLED_Clear();
  while (1)
  {
    /* 烧录成功测试：每500ms通过UART4发送一次 666，串口助手可随时捕获 */
    if (HAL_GetTick() - lastTick >= 500)
    {
      lastTick = HAL_GetTick();
      UART4_Printf("666\r\n");
      HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);   // 诊断LED0: 主循环心跳(500ms翻转)
    }

    /* ============ 四轮同转测试: 初始化时已把四个 PWM 置为 +MOTOR_TEST_PWM(200=25%),
     * 开环控制环(motor_openloop=1)每10ms会把这四个 PWM 重新输出到 TB6612,
     * 所以四轮会一直同时正转, 主循环无需再改动任何值 ============ */
    /* ================================================================ */

    /* ================= OLED 调试显示：四轮同转状态 + IN电平 + 速度 + PWM + 编码器 =================
     * 第一行: ALL+25% = 四轮同时正转 25% 占空比;  n = 10ms控制环运行计数
     * 第二行: I:xxxx xxxx xxxx xxxx = 四路IN引脚(IN1IN2), 正转应为 01
     * v1~v4: 四轮实测速度(cm/s)，由编码器换算得到
     * P1~P4: 当前各电机 PWM(带符号, 符号=IN状态)
     * e1~e4: 四路编码器原始10ms增量(轮子转时应有非0值, 否则编码器没计数)
     * ====================================================================== */
    if (HAL_GetTick() - oledTick >= 300)
    {
      oledTick = HAL_GetTick();
      OLED_Printf(0, 0,  OLED_8X16, "ALL+25%% n:%lu", (unsigned long)motor_loop_count);
      /* I: 四路IN引脚实时电平(IN1IN2), 正转应为 "01 01 01 01" (IN1=0,IN2=1)
         * 某轮显示非01 -> MCU/GPIO问题; 显示01但轮子不转 -> 问题在TB6612模块/排座/电机 */
      OLED_Printf(0, 16, OLED_6X8, "I:%d%d %d%d %d%d %d%d",
        HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3), HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4),  // 电机1 AIN1 AIN2
        HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4), HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5),  // 电机2 BIN1 BIN2
        HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0), HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1),  // 电机3 CIN1 CIN2
        HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2), HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7)); // 电机4 DIN1 DIN2
      OLED_Printf(0, 24, OLED_6X8, "v1:%5.0f v2:%5.0f", (double)my_car.motor_1.speed, (double)my_car.motor_2.speed);
      OLED_Printf(0, 32, OLED_6X8, "v3:%5.0f v4:%5.0f", (double)my_car.motor_3.speed, (double)my_car.motor_4.speed);
      /* P: 四轮PWM(带符号, 符号=IN状态)   e: 四路编码器原始10ms增量(轮子转时应有非0值) */
      OLED_Printf(0, 40, OLED_6X8, "P:%3ld %3ld %3ld %3ld",
        (long)my_car.motor_1.PWM, (long)my_car.motor_2.PWM,
        (long)my_car.motor_3.PWM, (long)my_car.motor_4.PWM);
      OLED_Printf(0, 48, OLED_6X8, "e:%4ld %4ld %4ld %4ld",
        (long)my_car.motor_1.encoder_count, (long)my_car.motor_2.encoder_count,
        (long)my_car.motor_3.encoder_count, (long)my_car.motor_4.encoder_count);
      /* A=编码器1(PA5,PB3) B=2(PA6,PA7) C=3(PD12,PD13) D=4(PA0,PA1)
         D 两位在轮4转动时应 0/1 交替变化; 一直不变=编码器信号没到芯片 */
      OLED_Printf(0, 56, OLED_6X8, "A:%d%dB:%d%dC:%d%dD:%d%d",
        (GPIOA->IDR >> 5) & 1, (GPIOB->IDR >> 3) & 1,
        (GPIOA->IDR >> 6) & 1, (GPIOA->IDR >> 7) & 1,
        (GPIOD->IDR >> 12) & 1, (GPIOD->IDR >> 13) & 1,
        (GPIOA->IDR >> 0) & 1, (GPIOA->IDR >> 1) & 1);
      OLED_Update();
    }
    /* ===================== OLED 调试显示结束 ===================== */

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
