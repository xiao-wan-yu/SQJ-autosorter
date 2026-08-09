#include <stdlib.h>
#include <stm32f4xx_hal.h>
#include <string.h>
#include "serialplot.h"
#include "pid.h"
#include "oled_api.h"
#include "uart.h"

/*此处声明可能会用到的变量，视具体情况调整*/
// extern PID_POS pid_speed;
// extern PID_POS pid_position;
extern PID_POS line;
extern PID_POS angle;
extern PID_POS speed_left;
extern PID_POS speed_right;

#define PARAM_Number 5              //参数个数
#define PID_Loop  angle         //要调参的pid环地址需

typedef struct{
  void *p;        //某一个参数变量的地址
  char *name;     //该参数变量的名字
}Param;

Param param[PARAM_Number] = { //可以修改的变量列表
  //example:  {&Kp, "Kp"},
  {&PID_Loop.kp, "kp"},
  {&PID_Loop.ki, "ki"},
  {&PID_Loop.kd, "kd"},
  {&PID_Loop.target, "target"},
  {&PID_Loop.target, "target_L"}
};

/**
  * @brief 接收串口绘图软件发送的参数修改指令，为快速调参PID而生
  * @param string 接收到的字符串指令 格式:"name type num "，其中name为变量名字，type为数据类型：f/i，num为需要修改的数值
  * @attention 禁止传入字符串，本函数会对传入字符数组进行修改！！！
  */
void SERIALPLOT_ChangeParam(char *string){
  char *str_name, *str_type, *str_num;  
  char *str_temp = string;
  void *p;  //目标参数的指针

  str_name = string;//把接收到的字符串的三个空格转换成'\0'并获取三个参数的位置
  for(int i=0; i <= 2; i++){  
    str_temp = strchr(str_temp, ' ');
    *str_temp = '\0';
    str_temp++;
    if(i == 0) str_type  = str_temp;
    else if(i == 1) str_num = str_temp;
  }

  for(int i=0; i<= PARAM_Number-1; i++){  //确定接收到的子串名字与哪个变量名字相对应
    if(strcmp(str_name, param[i].name) == 0){ 
      p = param[i].p;
    }
  }

  char *end_ptr;  //根据指定的数据类型，把接收到的数据子串转换为对应类型并存储到对应变量中
  switch(*str_type){
    case 'f': //数值为浮点数
      *(float *)p = strtof(str_num, &end_ptr);
      break;
    case 'i': //数值为整数
      *(int *)p  = strtol(str_num, &end_ptr, 0);
      break;
    default:
      break;
  }
}

/**
  * @brief 利用serialplot画图软件进行PID调参
  */
void SERIALPLOT_PIDAdjustParam(void){
  while(1){
    OLED_Printf(0, 0, OLED_8X16, "kp:%06.2f", PID_Loop.kp);
    OLED_Printf(0, 16, OLED_8X16, "ki:%06.2f", PID_Loop.ki);
    OLED_Printf(0, 32, OLED_8X16, "kd:%06.2f", PID_Loop.kd);
    OLED_Printf(0, 48, OLED_8X16, "tar:%06.2f", PID_Loop.target);
    OLED_Update();
    UART1_Printf("%f %f %f %f\r\n", PID_Loop.target, PID_Loop.actual, PID_Loop.out, PID_Loop.errorint);
    if(UART1_RxFlag){
      UART1_RxFlag = 0;
      SERIALPLOT_ChangeParam((char *)UART1_RxBuf);
    }
  }
}
