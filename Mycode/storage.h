#ifndef __STORAGE_H
#define __STORAGE_H

#include <stdint.h>

#define STORAGE_START_ADDRESS		0x08060000 		              //存储的起始地址（该地址为扇区7的起始地址）
#define STORAGE_Flag            0xA5A52424                  //判断之前是否存储过数据，存储过的话在指定位置有这个数字
                                                          //★2026.10.6：结构体加了比赛流程参数(布局变了) ⇒ 标志位换代，
                                                          //  原来 Flash 里的旧数据自动作废，第一次上电会写入标称值
                                                          //  (旧值 0xA5A52323；要保留旧数据就改回去，但旧数据的偏移量对不上)
#define STORAGE_COUNT				    sizeof(STORAGE_TYPE) / 4	  //存储数据的个数（包含标志位）

typedef struct{
  uint32_t flag;  //标志位
  /*下面为真实数据*/
  /* ==================== 第一部分：比赛流程用的“距离/时间”参数（队友 2026.10.2 新增，全部可在上电 KEY0 菜单里调） ====================
     ★每一项后面注释里的数字 = 2026.10.2 之前的硬编码值，也就是“默认/标称值”
       （main.c 里 ROBO_LoadDefaultParams() 填的就是这些，改默认值要改那个函数）。
     ★这些字段被 main.c 直接使用，改结构体顺序/名字前先搜一遍用法。
     ★改了字段/顺序 ⇒ 顺手把上面的 STORAGE_Flag 换代，否则旧 Flash 数据偏移量对不上。 */
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

  int32_t R_LiZhuang_Delay_x;  //红方-立柱-x延迟时间  120
  int32_t B_LiZhuang_Delay_x;  //蓝方-立柱-x延迟时间  150
  int32_t R_LiZhuang_Mid_y;  //红方-立柱-抓中间方块目标y距离  175
  int32_t B_LiZhuang_Mid_y;  //蓝方-立柱-抓中间方块目标y距离  175
  int32_t R_LiZhuang_Cir_y;  //红方-立柱-绕圈目标y距离  145
  int32_t B_LiZhuang_Cir_y;  //蓝方-立柱-绕圈目标y距离  145

  int32_t R_YuanHuan_Delay_x;  //红方-圆环-x延迟时间  440
  int32_t B_YuanHuan_Delay_x;  //蓝方-圆环-x延迟时间  320
  int32_t R_YuanHuan_Target_y;  //红方-圆环-目标y距离  55
  int32_t B_YuanHuan_Target_y;  //蓝方-圆环-目标y距离  55

  int32_t R_HuiJia_x;  //红方-回家-x移动距离  12
  int32_t B_HuiJia_x;  //蓝方-回家-x移动距离  -31
  int32_t C_HuiJia_y;  //共同-回家-y移动距离  -6

}STORAGE_TYPE;//SRAM结构体类型

extern STORAGE_TYPE STORAGE_Data; 

void STORAGE_Init(void);
void STORAGE_Save(void);
void STORAGE_Clear(void);

#endif
