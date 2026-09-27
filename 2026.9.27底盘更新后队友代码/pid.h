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
  float errorint;    //误差积分（★注意：是"误差的累加"，不是"积分项输出"；积分项输出 = ki*errorint）
  float integral_max;//积分限幅上限（积分超过该值会被截断；若置 0 则不做积分限幅）
  /*输出上下限*/
  float out_max;  //输出上限
  float out_min;  //输出下限
  /* ★ 分项输出（2026-09-25 加，仅供串口/OLED 观察是哪一项在起作用，不参与任何运算）
     用途：调速度环时判断"现在是 P 顶着、还是 I 在爬"——积分饱和、防积死、超调都靠它定位 */
  float p_out;    //比例项输出 = kp*error0
  float i_out;    //积分项输出 = ki*errorint
  float d_out;    //微分项输出 = kd*(error0-error1)
  /* ★ 积分使能条件（2026-09-25 加，只有 PID_PosSpeedUpdate 读它们；置 0 = 该条件失效）
     三个条件任一不满足 → 本周期不积分。全部置 0 时等价于"无条件积分"，即最朴素的位置式 */
  float ki_eps;      //|ki| < 该值 → 视为 ki=0：不积分 且 把 errorint 清 0（默认 0.001）
  float int_sep_th;  //积分分离：|error0| > 该值 → 本周期不积分（防大误差下积分饱和导致超调，0=关闭）
  float int_dz;      //积分死区：|error0| < 该值 → 本周期不积分（防小误差下积分抖振/来回蹭，0=关闭）
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
void PID_PosSpeedUpdate(PID_POS *pid);   // 位置式**速度环**专用（2026-09-25 加）：比 PID_PosUpdate 多了
                                         // 分项输出 p_out/i_out/d_out，以及三个可调的积分使能条件
                                         // （ki_eps / int_sep_th / int_dz）。原 PID_PosUpdate 保持不动。
void PID_IncUpdate(PID_INC *pid);
void PID_Line(PID_POS *pid);
void PID_Angle(PID_POS *pid);
void PID_Speed(PID_POS *pid);

#endif
