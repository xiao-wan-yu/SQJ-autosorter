#ifndef __STORAGE_H
#define __STORAGE_H

#include <stdint.h>

#define STORAGE_START_ADDRESS		0x08060000 		              //存储的起始地址（该地址为扇区7的起始地址）
#define STORAGE_Flag            0xA5A52323                  //判断之前是否存储过数据，存储过的话在指定位置有这个数字
#define STORAGE_COUNT				    sizeof(STORAGE_TYPE) / 4	  //存储数据的个数（包含标志位）

typedef struct{
  uint32_t flag;  //标志位
  /*下面为真实数据*/

  int32_t R_ChuFa_x;  //红方-出发-x移动距离 -88
  int32_t R_ChuFa_y;  //红方-出发-y移动距离 424
  int32_t B_ChuFa_x;  //蓝方-出发-x移动距离 50
  int32_t B_ChuFa_y;  //蓝方-出发-y移动距离 405

  int32_t R_YuanPan_y;  //红方-圆盘机-y后退距离 -6
  int32_t B_YuanPan_y;  //蓝方-圆盘机-y后退距离 -6

  int32_t C_GetTarget_y;  //共同-到达识别目标字母位置的y移动距离 129
  int32_t C_GetTarget_x;  //共同-到达识别目标字母位置的x移动距离 -42

  int32_t C_JieTi_Left_x;  //共同-阶梯-最左侧到第一个坑位x移动距离  7
  int32_t C_Jieti_FwdTarget_y;  //共同-阶梯-前面测距目标距离 55
  int32_t C_JitTi_Step_x;  //共同-阶梯-同阶梯相邻坑间距 16
  int32_t C_Jieti_Cross_x;  //共同-阶梯-换阶梯那一步(第2→3、第6→7个坑) 20
  int32_t C_JieTi_JiaoZhun_x1;  //共同-阶梯-校准x左部容差 -5
  int32_t C_JieTi_JiaoZhun_x2;  //共同-阶梯-校准x右部容差 18
  int32_t C_JieTi_JiaoZhun_y1;  //共同-阶梯-校准y前部容差 -5
  int32_t C_JieTi_JiaoZhun_y2;  //共同-阶梯-校准y后部容差 +5

  int32_t R_LiZhuang_Delay_x;  //红方-立桩-x延迟时间  120
  int32_t B_LiZhuang_Delay_x;  //蓝方-立桩-x延迟时间  150
  int32_t C_LiZhuang_Target_y;  //共同-立桩-目标y距离  175
  int32_t C_LiZhuang_Fwd_y;  //共同-立桩-y前进距离  6
  int32_t R_LiZhuang_Back_y;  //红方-立桩-y后退距离  -8
  int32_t B_LiZhuang_Back_y;  //蓝方-立桩-y后退距离  -9

  int32_t R_HuiJia_x;  //红方-回家-x移动距离  11
  int32_t B_HuiJia_x;  //蓝方-回家-x移动距离  -33
  int32_t C_HuiJia_y;  //共同-回家-y移动距离  -6

}STORAGE_TYPE;//SRAM结构体类型

extern STORAGE_TYPE STORAGE_Data; 

void STORAGE_Init(void);
void STORAGE_Save(void);
void STORAGE_Clear(void);

#endif
