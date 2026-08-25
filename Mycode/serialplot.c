#include <stdlib.h>
#include <stm32f4xx_hal.h>
#include <string.h>
#include "serialplot.h"
#include "oled_ui/oled.h"
#include "pid.h"
#include "oled_api.h"
#include "uart.h"
#include "chassis.h"
#include "hwt101ct.h"

#define PARAM_Number 9              //参数个数（航向环 kp/ki/kd/target + 整车速度 vx/vy/w + 规划 mv/mvacc）
#define YAW_Loop  chassis.yaw_pid   //要调参的pid环：整车航向环
/* 死区无需在线调：控制循环按是否平移自动切换 —— 静止旋转 YAW_DEAD_ZONE_TURN(1.0°) 防来回飘，
   走直线(有平移) YAW_DEAD_ZONE_MOVE(0.3°) 让1°内偏航也被纠正 */

typedef struct{
  void *p;        //某一个参数变量的地址
  char *name;     //该参数变量的名字
}Param;

Param param[PARAM_Number] = { //可以修改的变量列表（名字匹配后按名字分发）
  {&YAW_Loop.kp, "kp"},        // 航向环比例（静止旋转-0.028；走直线纠偏可调大如-0.05）
  {&YAW_Loop.ki, "ki"},        // 航向环积分（实测0即可）
  {&YAW_Loop.kd, "kd"},        // 航向环微分（实测0即可，速度环自带阻尼）
  {&chassis.target_yaw, "target"}, // 航向环目标角度（0~360，遥控转向）
  {&chassis.v_x, "vx"},        // 整车x速度 cm/s（右移为正；手动设速模式，见 ChangeParam）
  {&chassis.v_y, "vy"},        // 整车y速度 cm/s（前进为正；手动设速模式）
  {&chassis.w,   "w"},         // 整车角速度 rad/s（航向环开启时被角度环接管，需 flag.angle=0 才直接生效）
  {&chassis.move_speed, "mv"},   // 梯形规划目标速度 cm/s（mx/my 走固定距离用，默认60）
  {&chassis.move_acc,   "mvacc"} // 梯形规划加减速 cm/s²（默认100）
};

/**
  * @brief 接收串口绘图软件发送的参数修改指令，为快速调参PID而生
  * @param string 接收到的字符串指令 格式:"name type num "，其中name为变量名字，type为数据类型：f/i，num为需要修改的数值
  * @note   指令示例：
  *         "kp f -0.028"  调航向环参数；"target f 90" 遥控转向；"vx f 30" 手动x横移（持续）
  *         "mx f 30"      x方向平移30cm自动停；"my f -50" y方向后退50cm自动停（梯形速度规划）
  * @attention 禁止传入字符串，本函数会对传入字符数组进行修改！！！
  */
void SERIALPLOT_ChangeParam(char *string){
  char *str_name, *str_type, *str_num;
  char *str_temp;

  /*先数空格：指令必须 2 个空格分隔 3 段（name type num），不足直接忽略。
    ！！！原实现找第3个空格时 strchr 返回 NULL 后 *str_temp='\0' 写地址0，会崩溃，已修*/
  str_temp = string;
  int sp = 0;
  while((str_temp = strchr(str_temp, ' ')) != NULL){ sp++; str_temp++; }
  if(sp < 2) return;

  str_name = string;
  str_temp = string;
  for(int i=0; i <= 1; i++){
    str_temp = strchr(str_temp, ' ');
    *str_temp = '\0';
    str_temp++;
    if(i == 0) str_type  = str_temp;
    else if(i == 1) str_num = str_temp;
  }

  char *end_ptr;
  float val = 0.0f;
  switch(*str_type){//根据指定的数据类型转换数值（PID参数均为float）
    case 'f': //数值为浮点数
      val = strtof(str_num, &end_ptr);
      break;
    case 'i': //数值为整数
      val = (float)strtol(str_num, &end_ptr, 0);
      break;
    default:
      break;
  }

  /* 梯形规划指令（mx/my 不在 param 表，单独分发）：mx f 30 → x方向平移30cm自动停；my 同理
     速度/加速度用 param 表默认值 mv/mvacc（距离0的轴速度/加速度传0即可，不参与规划） */
  if(strcmp(str_name, "mx") == 0){ CHASSIS_Start_Move(val, 0.0f, chassis.move_speed, 0.0f, chassis.move_acc, 0.0f); return; }
  if(strcmp(str_name, "my") == 0){ CHASSIS_Start_Move(0.0f, val, 0.0f, chassis.move_speed, 0.0f, chassis.move_acc); return; }

  for(int i=0; i<= PARAM_Number-1; i++){  //确定接收到的子串名字与哪个变量名字相对应
    if(strcmp(str_name, param[i].name) == 0){
      *(float*)param[i].p = val;   // 直接写入对应变量（kp/ki/kd/target/w/mv/mvacc；vx/vy 先写值再设标志）
      /* vx/vy 为手动设速模式：置对应轴 set_speed_flag（中断不清零该轴 v_），并取消该轴正在执行的规划 */
      if(strcmp(str_name, "vx") == 0){
        chassis.x_set_speed_flag  = 1;
        chassis.x_speed_plan_flag = 0;
      }else if(strcmp(str_name, "vy") == 0){
        chassis.y_set_speed_flag  = 1;
        chassis.y_speed_plan_flag = 0;
      }
      break;
    }
  }
}

/**
  * @brief 利用serialplot画图软件进行PID调参
  * @note  串口发送5通道数据：目标速度 + 4轮实测速度(空格分隔+\r\n)，SerialPlot画图
  *        调参指令格式：名字 类型 数值，如 "kp f 15" / "target f 30"
  */
void SERIALPLOT_PIDAdjustParam(void){
  /*整车速度清零：车静止只做转向，w 由航向环接管（角度环自动算 w）*/
  chassis.v_x = 0.0f;
  chassis.v_y = 0.0f;
  while(1){
    /* OLED：显示目标/实际角度 + 误差/输出w + 角度环参数 + 里程计位置 */
    OLED_Printf(0, 0,  OLED_8X16_HALF, "tar:%05.2f yaw:%05.2f", chassis.target_yaw, HWT101CT_Data.yaw);
    OLED_Printf(0, 16, OLED_8X16_HALF, "e:%+05.2f w:%+04.1f", chassis.yaw_pid.error0, chassis.yaw_pid.out);
    OLED_Printf(0, 32, OLED_8X16_HALF, "kp:%05.2f kd:%05.2f", chassis.yaw_pid.kp, chassis.yaw_pid.kd);
    OLED_Printf(0, 48, OLED_8X16_HALF, "x:%05.2f y:%05.2f", chassis.pos_x, chassis.pos_y);
    OLED_Update();
    /*5通道：目标角度 + 实际角度 + 角度环输出w + 里程计位置x/y，SerialPlot 观察转向收敛与位置累积*/
    UART1_Printf("%f %f %f %f %f\r\n",
                 chassis.target_yaw,
                 HWT101CT_Data.yaw,
                 chassis.yaw_pid.out,
                 chassis.pos_x,
                 chassis.pos_y);
    if(UART1_RxFlag){
      UART1_RxFlag = 0;
      SERIALPLOT_ChangeParam((char *)UART1_RxBuf);
    }
    HAL_Delay(10);
  }
}
