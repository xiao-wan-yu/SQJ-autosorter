#include <stm32f4xx_hal.h>
#include "encoder.h"
#include "stm32f4xx_hal_tim.h"

extern TIM_HandleTypeDef htim2; //左前车轮编码器
extern TIM_HandleTypeDef htim3; //左后轮编码器
extern TIM_HandleTypeDef htim4; //右后轮编码器
extern TIM_HandleTypeDef htim5; //右前车轮编码器

/**
  * @brief 获取单位时间内的编码器脉冲数（清零法：读 CNT 后立即清零）
  * @param ENCODER_Num 编码器号码 可选择：ENCODER_LeftFront ENCODER_LeftBack ENCODER_RightBack ENCODER_RightFront
  * @retval 返回自上次读取至今的脉冲增量，可正可负
  * @note  清零法优点：无需 16 位定时器(TIM3/TIM4)溢出中断。控制周期 10ms 内最高速累计也只有几十个脉冲，
  *        计数器永远不会溢出(65535)，因此读取后清零安全且不依赖任何中断，杜绝了中断函数缺失导致的卡死。
  *        取 (int16_t) 低16位即可得到带符号增量（10ms 内 |增量| << 32768，正反方向都正确）。
  */
int16_t ENCODER_GetPulse(uint8_t ENCODER_Num){
  TIM_HandleTypeDef *htim;
  switch(ENCODER_Num){
    case ENCODER_LeftFront:  htim = &htim2; break;
    case ENCODER_LeftBack:   htim = &htim3; break;
    case ENCODER_RightBack:  htim = &htim4; break;
    case ENCODER_RightFront: htim = &htim5; break;
    default: return 0;
  }
  int16_t pulse = (int16_t)__HAL_TIM_GET_COUNTER(htim);  //读 CNT（低16位即带符号增量）
  __HAL_TIM_SET_COUNTER(htim, 0);                        //读后清零
  return pulse;
}
