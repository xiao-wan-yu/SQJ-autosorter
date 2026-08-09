#include "stm32f4xx_hal.h"
#include "key.h"
#include "main.h"
#include "delay.h"

/**
  * @brief 检测某个按键是否被按下
  * @note 直到松手才退出函数
	* @retval 按键被按下返回1 没被按下返回0
  */
uint8_t KEY_ONE(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
	uint8_t a = 0;
	if(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin)==GPIO_PIN_RESET){
		delay_us(20000);//延时去抖动
		if(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin)==GPIO_PIN_RESET){
			a=1;//进入按键处理，返回1
		}
	}
	while(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin)==GPIO_PIN_RESET); //等待按键松开
	return a;
}


/**
	* @brief 检测所有按键中是否有按键被按下
	* @note 直到松手才退出函数，一次最多只能检测到一个按键，要避免多个按键同时按下
	* @retval 有按键按下返回键值，没按键按下返回KEY_NO
	*
	*/
int8_t KEY_ALL(void){
  int8_t KEYNum  = KEY_NO;
  if(KEY_ONE(KEY0_GPIO_Port, KEY0_Pin)) KEYNum = 0;
  if(KEY_ONE(KEY1_GPIO_Port, KEY1_Pin)) KEYNum = 1;
  if(KEY_ONE(KEY2_GPIO_Port, KEY2_Pin)) KEYNum = 2;
  if(KEY_ONE(KEY3_GPIO_Port, KEY3_Pin)) KEYNum = 3;
  return KEYNum;
}
