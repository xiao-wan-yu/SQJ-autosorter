#include "vision.h"

VISION_DATA VISION_Data; 

/**
  * @brief  解析视觉模块信息并存入 VisionData 结构体
  */
void VISION_ReceiveData(uint8_t *buf, uint8_t buf_len)
{
  //首先判断数据包是否接收成功
  if((buf[0] == 0xA1 || buf[0] == 0xA2 || buf[0] == 0xA3)//包头正确
    && (buf[buf_len-1] == 0x0B)//包尾正确
    && (buf_len <= 7)//数据长度正确
  ){
    VISION_Data.success = 1; //接收成功
  }
  else{
    VISION_Data.success = 0; //接收失败
    return;
  }

  //更新VISION_Data变量信息
  VISION_Data.period = buf[0] - 0xA0; //当前视觉工作的阶段（圆盘机/阶梯/立桩）
  VISION_Data.target = buf[1];        //是否是需要抓取的目标物
  if(VISION_Data.period == 1){//圆盘机阶段，距离信息无效
    VISION_Data.x = (uint16_t)buf[2] | (uint16_t)buf[3] << 8; //目标物在图像中的x坐标（像素）
    VISION_Data.y = (uint16_t)buf[4] | (uint16_t)buf[5] << 8; //目标物在图像中的y坐标（像素）
  }
  else if(VISION_Data.period == 2){
    VISION_Data.distance = (uint16_t)buf[2] | (uint16_t)buf[3] << 8; //目标物在图像中的距离
  }
}

