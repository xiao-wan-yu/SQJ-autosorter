#ifndef __PID_H
#define __PID_H

typedef struct{
  /*输入输出值*/
  float target;   //目标值
  float actual;   //实际值
  float out;      //输出值
  /*PID参数*/
  float kp;       //Kp
  float ki;       //Ki
  float kd;       //Kd
  /*中间变量*/
  float error0;   //本次误差
  float error1;   //上次误差
  float errorint; //误差积分
  /*输出上下限*/
  float out_max;  //输出上限
  float out_min;  //输出下限
}PID_POS;  //位置环PID类型--一个位置环PID对应一个该类型的变量

void PID_PosUpdate(PID_POS *pid);
void PID_Line(PID_POS *pid);
void PID_Angle(PID_POS *pid);
void PID_Speed(PID_POS *pid);

#endif
