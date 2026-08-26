#ifndef __VISION_H
#define __VISION_H

#include <stm32f4xx_hal.h>

typedef struct{
  uint8_t success;         // 接收数据是否成功（数据包是否正常）
  uint8_t period;          // 当前视觉工作的阶段（圆盘机/阶梯/立桩）
  uint8_t target;          // 是否是需要抓取的目标物
  uint16_t x;              // 目标物在图像中的x坐标（像素）
  uint16_t y;              // 目标物在图像中的y坐标（像素）
  uint16_t distance;       // 目标物在图像中的距离
  
}VISION_DATA;

extern VISION_DATA VISION_Data; 


void VISION_ReceiveData(uint8_t *buf, uint8_t buf_len);


#endif
