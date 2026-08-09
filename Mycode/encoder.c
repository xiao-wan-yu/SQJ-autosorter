#include <stm32f4xx_hal.h>
#include "encoder.h"
#include "stm32f4xx_hal_tim.h"

extern TIM_HandleTypeDef htim2; //左车轮编码器
extern TIM_HandleTypeDef htim3; //右车轮编码器

#if ENCODER_USE_LeftTotal
int32_t encoder_lefttotal = 0;
#endif

#if ENCODER_USE_RightTotal
int32_t encoder_righttotal = 0;
#endif

/**
  * @brief 获取某个时间段内的编码器脉冲数并清零
  * @param ENCODER_Num 编码器号码 可选择：ENCODER_Left或ENCODER_Right
  * @retval 返回某个时间段内的编码器脉冲数，可正可负
  * @attention 在电赛中一般是定时获取单位时间脉冲数，如定时10ms获取一次
  */
int16_t ENCODER_GetPulse(uint8_t ENCODER_Num){
  int16_t cnt = 0;
  switch(ENCODER_Num){
    case ENCODER_Left:
      cnt = __HAL_TIM_GET_COUNTER(&htim2);
      __HAL_TIM_SET_COUNTER(&htim2, 0);
      #if ENCODER_USE_LeftTotal //左车轮编码器计数总和
      encoder_lefttotal += cnt;
      #endif
      break;
    case ENCODER_Right:
      cnt = __HAL_TIM_GET_COUNTER(&htim3);
      __HAL_TIM_SET_COUNTER(&htim3, 0);
      #if ENCODER_USE_RightTotal //右车轮编码器计数总和
      encoder_righttotal += cnt;
      #endif
      break;
    default:
      break;
  }
  return cnt;
}

