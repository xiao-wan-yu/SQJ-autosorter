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

	/*快速初检：未按下时立即返回，保持零开销*/
	if(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) != GPIO_PIN_RESET){
		return 0;
	}

	/*初检为按下：连续采样，需连续3次(间隔5ms)读到低电平才确认按下
	  消除机械按键弹跳导致的误判（弹跳期间的偶发高电平会重置计数）*/
	uint8_t stable = 0;
	for(uint8_t i = 0; i < 6; i++){  //最多采样6次（30ms窗口）
		delay_us(5000);
		if(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == GPIO_PIN_RESET){
			if(++stable >= 3){  //连续3次低电平 → 确认按下
				a = 1;
				break;
			}
		}else{
			stable = 0;  //读到高电平（弹跳），重置连续计数
		}
	}

	if(a == 1){
		/*等待按键松开*/
		while(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == GPIO_PIN_RESET);
		/*松手消抖，防止松手弹跳被误判为新一次按下*/
		delay_us(20000);
	}

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
