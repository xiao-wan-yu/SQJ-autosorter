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

  /* ========== 四轮开环正转：20% 占空比，一直转 ==========
   * 速度值即占空比百分比：20 -> 20%（TIM1 ARR=799，比较值 = 20*800/100 = 160）
   * 注意：核心板蜂鸣器在 PB0(=CIN1)，C 电机正转走 CIN2 故蜂鸣器不响；
   *       若某电机转向相反，对调该路电机两根线即可。
   */
  TB6612_Init();                    // STBY=1 退出待机 + 启动 TIM1 四路 PWM
  TB6612_Control(MOTOR_A, 80);      // A电机 20% 占空比
  TB6612_Control(MOTOR_B, 80);      // B电机 20% 占空比
  TB6612_Control(MOTOR_C, 80);      // C电机 20% 占空比（走CIN2，蜂鸣器不响）
  TB6612_Control(MOTOR_D, 80);      // D电机 20% 占空比


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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 四路电机已在初始化后以 20% 占空比开环正转，主循环保持运行即可 */
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
