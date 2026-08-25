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
  float error0;      //本次误差
  float error1;      //上次误差
  float errorint;    //误差积分
  float integral_max;//积分限幅上限（积分超过该值会被截断；若置 0 则不做积分限幅）
  /*输出上下限*/
  float out_max;  //输出上限
  float out_min;  //输出下限
}PID_POS;  //位置环PID类型--一个位置环PID对应一个该类型的变量

/*=================== 增量式PID ===================*/
/* 执行逻辑照搬旧代码 IncrementalPID_Calculate（去年调好的手感算法），仅做命名/结构清晰化 */
typedef struct{
  /*输入输出值*/
  float target;    //目标值 cm/s
  float actual;    //实际值 cm/s
  float out;       //PWM输出（增量累加，限幅±out_max）
  /*PID参数*/
  float kp;        //Kp
  float ki;        //Ki
  float kd;        //Kd
  /*分项增量*/
  float p_out;     //比例项增量
  float i_out;     //积分项（累计，限幅±10）
  float d_out;     //微分项增量
  /*误差历史*/
  float err;       //本次误差
  float last_err;  //上次误差
  float prev_err;  //上上次误差
  /*输出限幅*/
  float out_max;   //输出对称限幅 ±out_max
}PID_INC;  //增量式PID类型--速度环

void PID_PosUpdate(PID_POS *pid);
void PID_IncUpdate(PID_INC *pid);
void PID_Line(PID_POS *pid);
void PID_Angle(PID_POS *pid);
void PID_Speed(PID_POS *pid);

#endif
