#ifndef __STORAGE_H
#define __STORAGE_H

#include <stdint.h>

#define STORAGE_START_ADDRESS		0x08060000 		              //存储的起始地址（该地址为扇区7的起始地址）
#define STORAGE_Flag            0xA5A52323                  //判断之前是否存储过数据，存储过的话在指定位置有这个数字
#define STORAGE_COUNT				    sizeof(STORAGE_TYPE) / 4	  //存储数据的个数（包含标志位）

typedef struct{
  uint32_t flag;  //标志位
  /*下面为真实数据*/
  float line_kp;  //循迹环参数
  float line_ki;
  float line_kd;
  float angle_kp;  //角度环参数
  float angle_ki;
  float angle_kd;
  float speed_left_kp;  //左轮速度环参数
  float speed_left_ki;
  float speed_left_kd;
  float speed_right_kp;  //右轮速度环参数
  float speed_right_ki;
  float speed_right_kd;

  float angle_offset; //偏移角度
  int32_t encoder_cnt_odd; //奇数次的编码器计数值
  int32_t encoder_cnt_even; //偶数次的编码器计数值

}STORAGE_TYPE;//SRAM结构体类型

extern STORAGE_TYPE STORAGE_Data; 

void STORAGE_Init(void);
void STORAGE_Save(void);
void STORAGE_Clear(void);

#endif
