/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED0_Pin GPIO_PIN_2
#define LED0_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_5
#define LED3_GPIO_Port GPIOE
#define KEY0_Pin GPIO_PIN_0
#define KEY0_GPIO_Port GPIOC
#define KEY1_Pin GPIO_PIN_1
#define KEY1_GPIO_Port GPIOC
#define KEY2_Pin GPIO_PIN_2
#define KEY2_GPIO_Port GPIOC
#define KEY3_Pin GPIO_PIN_3
#define KEY3_GPIO_Port GPIOC
#define AIN1_Pin GPIO_PIN_3
#define AIN1_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_4
#define AIN2_GPIO_Port GPIOA
#define BIN1_Pin GPIO_PIN_4
#define BIN1_GPIO_Port GPIOC
#define BIN2_Pin GPIO_PIN_5
#define BIN2_GPIO_Port GPIOC
#define CIN1_Pin GPIO_PIN_0
#define CIN1_GPIO_Port GPIOB
#define CIN2_Pin GPIO_PIN_1
#define CIN2_GPIO_Port GPIOB
#define DIN1_Pin GPIO_PIN_2
#define DIN1_GPIO_Port GPIOB
#define DIN2_Pin GPIO_PIN_7
#define DIN2_GPIO_Port GPIOE
#define TB6612_STBY_Pin GPIO_PIN_8
#define TB6612_STBY_GPIO_Port GPIOE
#define OLED_SCL_Pin GPIO_PIN_13
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_15
#define OLED_SDA_GPIO_Port GPIOB
#define GY53_2_Pin GPIO_PIN_6
#define GY53_2_GPIO_Port GPIOC
#define GY53_1_Pin GPIO_PIN_7
#define GY53_1_GPIO_Port GPIOC
#define LASER2_Pin GPIO_PIN_8
#define LASER2_GPIO_Port GPIOC
#define LASER1_Pin GPIO_PIN_9
#define LASER1_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
/* ========== 临时：旧板引脚宏（仅为让旧 Mycode 编译通过，未接入新板，后续适配后删除）========== */
#define BUZZER_Pin GPIO_PIN_0
#define BUZZER_GPIO_Port GPIOB
#define GRAY1_Pin GPIO_PIN_8
#define GRAY1_GPIO_Port GPIOE
#define GRAY2_Pin GPIO_PIN_9
#define GRAY2_GPIO_Port GPIOE
#define GRAY3_Pin GPIO_PIN_10
#define GRAY3_GPIO_Port GPIOE
#define GRAY4_Pin GPIO_PIN_11
#define GRAY4_GPIO_Port GPIOE
#define GRAY5_Pin GPIO_PIN_12
#define GRAY5_GPIO_Port GPIOE
#define GRAY6_Pin GPIO_PIN_13
#define GRAY6_GPIO_Port GPIOE
#define GRAY7_Pin GPIO_PIN_14
#define GRAY7_GPIO_Port GPIOE
#define GRAY8_Pin GPIO_PIN_15
#define GRAY8_GPIO_Port GPIOE
#define MPU6050_SCL_Pin GPIO_PIN_12
#define MPU6050_SCL_GPIO_Port GPIOB
#define MPU6050_SDA_Pin GPIO_PIN_14
#define MPU6050_SDA_GPIO_Port GPIOB
#define ENCODER_LeftA_Pin GPIO_PIN_0
#define ENCODER_LeftA_GPIO_Port GPIOA
#define ENCODER_LeftB_Pin GPIO_PIN_1
#define ENCODER_LeftB_GPIO_Port GPIOA
#define ENCODER_RightA_Pin GPIO_PIN_6
#define ENCODER_RightA_GPIO_Port GPIOA
#define ENCODER_RightB_Pin GPIO_PIN_7
#define ENCODER_RightB_GPIO_Port GPIOA
#define SENSOR1_Pin GPIO_PIN_0
#define SENSOR1_GPIO_Port GPIOC
#define SENSOR2_Pin GPIO_PIN_1
#define SENSOR2_GPIO_Port GPIOC
#define SENSOR3_Pin GPIO_PIN_2
#define SENSOR3_GPIO_Port GPIOC
#define SENSOR4_Pin GPIO_PIN_3
#define SENSOR4_GPIO_Port GPIOC
#define NMOS1_Pin GPIO_PIN_8
#define NMOS1_GPIO_Port GPIOD
#define NMOS2_Pin GPIO_PIN_9
#define NMOS2_GPIO_Port GPIOD
#define TB6612_AIN1_Pin GPIO_PIN_12
#define TB6612_AIN1_GPIO_Port GPIOA
#define TB6612_AIN2_Pin GPIO_PIN_2
#define TB6612_AIN2_GPIO_Port GPIOB
#define TB6612_PWMA_Pin GPIO_PIN_8
#define TB6612_PWMA_GPIO_Port GPIOA
#define TB6612_PWMB_Pin GPIO_PIN_11
#define TB6612_PWMB_GPIO_Port GPIOA
#define TB6612_BIN1_Pin GPIO_PIN_10
#define TB6612_BIN1_GPIO_Port GPIOB
#define TB6612_BIN2_Pin GPIO_PIN_11
#define TB6612_BIN2_GPIO_Port GPIOB
#define ICM42688_CS_Pin GPIO_PIN_4
#define ICM42688_CS_GPIO_Port GPIOB
/* ========== 临时宏结束 ========== */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
