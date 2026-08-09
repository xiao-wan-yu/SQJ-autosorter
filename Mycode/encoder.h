#ifndef __ENCODER_H
#define __ENCODER_H

#define ENCODER_Left    1
#define ENCODER_Right   2

//编码器开启左车轮计数总和时置1
#define ENCODER_USE_LeftTotal 1
//编码器开启右车轮计数总和时置1
#define ENCODER_USE_RightTotal 1

int16_t ENCODER_GetPulse(uint8_t ENCODER_Num);


/******************当编码器开启左车轮计数总和时启用下面的宏定义********************/
#if ENCODER_USE_LeftTotal
extern int32_t encoder_lefttotal;
#endif
/***********************************************************************/

/******************当编码器开启右车轮计数总和时启用下面的宏定义********************/
#if ENCODER_USE_RightTotal
extern int32_t encoder_righttotal;
#endif
/***********************************************************************/



#endif
