#include "tb6612.h"
#include "main.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"

extern TIM_HandleTypeDef htim1;

/**
  * @brief TB6612 初始化：退出待机模式（STBY=1）并启动 TIM1 四路 PWM 输出
  * @note  需在外设初始化（MX_TIM1_Init / MX_GPIO_Init）之后调用
  */
void TB6612_Init(void)
{
  /* 退出待机模式，使能 TB6612 */
  HAL_GPIO_WritePin(TB6612_STBY_GPIO_Port, TB6612_STBY_Pin, GPIO_PIN_SET);

  /* 启动 4 路 PWM 输出（TIM1 是先进定时器，PWM_Start 会打开主输出 MOE） */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  /* 初始占空比清零 */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
}

/**
  * @brief 控制直流减速电机的转向和转速（四路 TB6612）
  * @param MOTOR_Num   电机通道：MOTOR_A / MOTOR_B / MOTOR_C / MOTOR_D
  * @param Motor_Speed 电机速度：-100 ~ 100（百分比）
  *        内部按 TIM1 自动重装值(ARR)自动换算成比较值：
  *        当前 ARR=799，精度 0~800，速度 20 即 20% 占空比 => 比较值 160。
  *        正值正转，负值反转。转向为约定方向，若实际转向相反，交换对应 IN1/IN2 电平即可。
  */
void TB6612_Control(uint8_t MOTOR_Num, int8_t Motor_Speed)
{
  uint32_t cmp;

  /* 限幅到 -100 ~ 100 */
  if (Motor_Speed > 100)  Motor_Speed = 100;
  if (Motor_Speed < -100) Motor_Speed = -100;

  /* 速度百分比 -> 0~ARR 档位（ARR=799，即 0~800 精度） */
  cmp = (uint32_t)(((int32_t)(Motor_Speed >= 0 ? Motor_Speed : -Motor_Speed))
                   * (int32_t)(__HAL_TIM_GET_AUTORELOAD(&htim1) + 1u) / 100);

  switch (MOTOR_Num)
  {
    case MOTOR_A:  /* A电机：PWMA=PE9(TIM1_CH1)，AIN1=PA3，AIN2=PA4
                      实测: AIN1=1,AIN2=0 能转（本通道唯一能转方向） */
      HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, (Motor_Speed >= 0) ? GPIO_PIN_SET   : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, (Motor_Speed >= 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, cmp);
      break;

    case MOTOR_B:  /* B电机：PWMB=PE11(TIM1_CH2)，BIN1=PC4，BIN2=PC5
                      实测: BIN1=0,BIN2=1 才能转（本通道唯一能转方向） */
      HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, (Motor_Speed >= 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
      HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, (Motor_Speed >= 0) ? GPIO_PIN_SET   : GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, cmp);
      break;

    case MOTOR_C:  /* C电机：PWMC=PE13(TIM1_CH3)，CIN1=PB0，CIN2=PB1
                      实测: CIN1=0,CIN2=1 才能转（本通道唯一能转方向）。
                      注意: CIN1=PB0 同时驱动核心板蜂鸣器，CIN1 = 1时蜂鸣器会响。 */
      HAL_GPIO_WritePin(CIN1_GPIO_Port, CIN1_Pin, (Motor_Speed >= 0) ? GPIO_PIN_RESET   : GPIO_PIN_SET);
      HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, (Motor_Speed >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, cmp);
      break;

    case MOTOR_D:  /* D电机：PWMD=PE14(TIM1_CH4)，DIN1=PB2，DIN2=PE7
                      实测: DIN1=0,DIN2=1 才能转（本通道唯一能转方向） */
      HAL_GPIO_WritePin(DIN1_GPIO_Port, DIN1_Pin, (Motor_Speed >= 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
      HAL_GPIO_WritePin(DIN2_GPIO_Port, DIN2_Pin, (Motor_Speed >= 0) ? GPIO_PIN_SET   : GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, cmp);
      break;

    default:
      break;
  }
}

