#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "gray.h"

uint8_t GRAY_Data[9];  //存储灰度传感器的值(下标为x 对应通道x)

/**
  * @brief 获取灰度传感器某一通道的值
  * @param Channel GRAYx 为灰度传感器的某一通道  x取值为1~8
  * @retval 返回指定通道的值 
  *         对应关系：颜色接近白色--灰度值高--灯亮--返回0
  *                  颜色接近黑色--灰度值低--灯灭--返回1
  */
GPIO_PinState  GRAY_ONE(uint8_t Channel){
    switch(Channel){
        case GRAY1:
            return HAL_GPIO_ReadPin(GRAY1_GPIO_Port, GRAY1_Pin);
        case GRAY2:
            return HAL_GPIO_ReadPin(GRAY2_GPIO_Port, GRAY2_Pin);
        case GRAY3:
            return HAL_GPIO_ReadPin(GRAY3_GPIO_Port, GRAY3_Pin);
        case GRAY4:
            return HAL_GPIO_ReadPin(GRAY4_GPIO_Port, GRAY4_Pin);
        case GRAY5:
            return HAL_GPIO_ReadPin(GRAY5_GPIO_Port, GRAY5_Pin);
        case GRAY6:
            return HAL_GPIO_ReadPin(GRAY6_GPIO_Port, GRAY6_Pin);
        case GRAY7:
            return HAL_GPIO_ReadPin(GRAY7_GPIO_Port, GRAY7_Pin);
        case GRAY8:
            return HAL_GPIO_ReadPin(GRAY8_GPIO_Port, GRAY8_Pin);
        default:
            return 2;   //这里或许得改一下
    }
}

/**
  * @brief 获取灰度传感器所有通道的值，并存储在gray_data数组中
  * @param Channel GRAYx 为灰度传感器的某一通道  x取值为1~8
  * @retval 返回指定通道的值 
  *         对应关系：颜色接近白色--灰度值高--灯亮--返回0
  *                  颜色接近黑色--灰度值低--灯灭--返回1
  */
void GRAY_ALL(void){
    GRAY_Data[1] = HAL_GPIO_ReadPin(GRAY1_GPIO_Port, GRAY1_Pin);
    GRAY_Data[2] = HAL_GPIO_ReadPin(GRAY2_GPIO_Port, GRAY2_Pin);
    GRAY_Data[3] = HAL_GPIO_ReadPin(GRAY3_GPIO_Port, GRAY3_Pin);
    GRAY_Data[4] = HAL_GPIO_ReadPin(GRAY4_GPIO_Port, GRAY4_Pin);
    GRAY_Data[5] = HAL_GPIO_ReadPin(GRAY5_GPIO_Port, GRAY5_Pin);
    GRAY_Data[6] = HAL_GPIO_ReadPin(GRAY6_GPIO_Port, GRAY6_Pin);
    GRAY_Data[7] = HAL_GPIO_ReadPin(GRAY7_GPIO_Port, GRAY7_Pin);
    GRAY_Data[8] = HAL_GPIO_ReadPin(GRAY8_GPIO_Port, GRAY8_Pin);
}




