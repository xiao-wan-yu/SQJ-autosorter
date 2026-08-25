#include <stdint.h>
#include <stm32f4xx_hal.h>
#include "pid.h"
#include "serialplot.h"
#include "tb6612.h"
// #include "gray.h"       //应用于PID循迹环
#include "hwt101ct.h"   //应用于PID角度环（HWT101CT 0~360°，逆时针/顺时针以实测为准）
#include "encoder.h"    //应用于PID速度环
#include <math.h>

/**
  * @brief 应用于PID中的限幅操作
  * @param data 要限幅变量的指针
  * @param ceiling 上限
  * @param floor 下限
  */
static void PID_Limit(float *data, float ceiling, float floor){
  if(*data > ceiling){
    *data = ceiling;
  }else if(*data < floor){
    *data = floor;
  }
}

/**
  * @brief 位置式PID更新
  * @param pid 包含某一环PID信息的变量指针
  * @attention 调用该函数前，应先更新pid->actual;调用该函数后，应利用pid->out控制执行器运动或传输给下一级PID
  *             该函数为模板函数，只完成了最基础的PID操作，想加入PID算法改进措施，可依据此模板为某一环PID定制函数
  */
void PID_PosUpdate(PID_POS *pid){
  //计算中间变量
  pid->error1 = pid->error0;
  pid->error0 = pid->target - pid->actual;
  if(fabs(pid->ki) > 0.001){
    pid->errorint += pid->error0;
  }else{
    pid->errorint = 0;
  }
  //积分限幅：防止误差长时间存在导致积分进入深度饱和
  if(pid->integral_max > 0.001f){
    PID_Limit(&pid->errorint, pid->integral_max, -pid->integral_max);
  }
  //进行PID运算
  pid->out = pid->kp*pid->error0 + pid->ki*pid->errorint + pid->kd*(pid->error0-pid->error1);
  //输出限幅
  PID_Limit(&pid->out, pid->out_max, pid->out_min);
}

/**
  * @brief 增量式PID更新（执行逻辑照搬旧代码 IncrementalPID_Calculate，仅命名/结构清晰化）
  * @param pid 包含某一环PID信息的变量指针
  * @attention 调用前更新 pid->actual；调用后利用 pid->out 控制执行器
  *            魔法数 10.0/0.1/0.2 为去年现场调好的手感值，勿改
  */
void PID_IncUpdate(PID_INC *pid){
  //计算误差
  pid->err = pid->target - pid->actual;

  //比例项增量
  pid->p_out = pid->kp * (pid->err - pid->last_err);
  //微分项增量
  pid->d_out = pid->kd * (pid->err - 2.0f*pid->last_err + pid->prev_err);

  //积分项累计 + 限幅±10（防止误差长时间存在导致积分深度饱和）
  pid->i_out += pid->ki * pid->err;
  if(pid->i_out > 10.0f)  pid->i_out =  10.0f;
  if(pid->i_out < -10.0f) pid->i_out = -10.0f;

  //防积死：误差微小时把积分项压缩到±0.2（避免低速时积分缓慢堆积、启动过冲）
  if(pid->ki*pid->err > -0.1f && pid->ki*pid->err < 0.1f){
    if(pid->i_out > 0.2f)       pid->i_out =  0.2f;
    else if(pid->i_out < -0.2f) pid->i_out = -0.2f;
  }

  //增量累加输出 + 限幅
  pid->out += pid->p_out + pid->i_out + pid->d_out;
  PID_Limit(&pid->out, pid->out_max, -pid->out_max);

  //误差递推
  pid->prev_err = pid->last_err;
  pid->last_err = pid->err;
}

/**
  * @brief PID角度环--采用位置式P(PD)控制器
  * @note 实际值的获取已封装在了函数中（读 HWT101CT_Data.yaw）；调用该函数后，利用 pid->out 作为整车角速度 w
  *       角度环PID参数（chassis 侧初始化）：kp=0.02 ki=0 kd=0.02 out_max=2.8 out_min=-2.8 执行周期=10ms
  *       Kp 符号以实测为准：本版按 w 逆时针为正、target>yaw 需逆时针转取正，方向反了取负
  */
void PID_Angle(PID_POS *pid){
  /*获取实际值，并对实际值加以优化，使小车可以选择角度更小的方向到达目标值（HWT101CT yaw 为 0~360°）*/
  if(HWT101CT_Data.yaw > pid->target + 180) pid->actual = HWT101CT_Data.yaw - 360;
  else if(HWT101CT_Data.yaw < pid->target - 180) pid->actual = HWT101CT_Data.yaw + 360;
  else pid->actual = HWT101CT_Data.yaw;
  /*计算中间变量*/
  pid->error1 = pid->error0;
  pid->error0 = pid->target - pid->actual;
  if(fabs(pid->ki) > 0.001) pid->errorint += pid->error0;
  else pid->errorint = 0;
  /*积分限幅：防止误差长时间存在导致积分进入深度饱和*/
  if(pid->integral_max > 0.001f){
    PID_Limit(&pid->errorint, pid->integral_max, -pid->integral_max);
  }
  /*pid运算*/
  pid->out = pid->kp*pid->error0 + pid->ki*pid->errorint + pid->kd*(pid->error0-pid->error1);
  /*输出限幅*/
  PID_Limit(&pid->out, pid->out_max, pid->out_min);
}

// /**
//   * @brief PID速度环--采用位置式PI控制器，加入了积分限幅
//   * @note 速度环PID参数参考值：kp=0.3 ki=0.03 target=？ out_max=100 out_min=-100 执行周期=2ms
//   * @attention 实际速度值范围约为-300~+300RPM；实际值的获取已封装在了函数中；调用该函数后，应利用pid->out控制执行器运动或传输给下一级PID
//   */
// void PID_Speed(PID_POS *pid){
//   /*电机参数配置*/
//   const float reduction_ratio = 28.0; //电机减速比
//   const uint8_t number_of_wires = 13; //电机线数
//   const uint8_t period = 2; //定时周期，单位：ms
//   const uint8_t encoder_multiple = 2; //编码器倍数，即一个脉冲周期被计数encoder_multiple次
//   extern PID_POS speed_left;
//   extern PID_POS speed_right;
//   //转速计算参考：pid->actual = (ENCODER_GetPulse(ENCODER_Right )/(float)(reduction_ratio*number_of_wires)) / (encoder_multiple*period/(1000*60.0));
//   static float speed_constant = (1000*60.0) / (float)((reduction_ratio*number_of_wires) * encoder_multiple*period);//转速常数--提前计算好系数，避免浪费时间重复计算
//   /*获取实际值--单位:RPM，计算公式：转数/分钟数*/
//   if(pid == &speed_right) 
//     pid->actual = ENCODER_GetPulse(ENCODER_Right ) * speed_constant;
//   else if(pid == &speed_left) 
//     pid->actual = ENCODER_GetPulse(ENCODER_Left ) * speed_constant;
//   //计算中间变量
//   pid->error1 = pid->error0;
//   pid->error0 = pid->target - pid->actual; 
//   pid->errorint += pid->error0;
//   PID_Limit(&pid->errorint, 4200, -4200);//积分限幅
//   //进行PID运算
//   pid->out = pid->kp*pid->error0 + pid->ki*pid->errorint + pid->kd*(pid->error0-pid->error1);
//   //输出限幅
//   PID_Limit(&pid->out, pid->out_max, pid->out_min);
// }








///*模板PID：位置式PID实现电机定速控制*/
// /*获取实际值*/
// actual = ENCODER_GetPulse(ENCODER_Left);
// /*计算中间变量*/
// error1 = error0;
// error0 = target - actual;
// // errorint += error0;
// if(fabs(Ki) < 0.0001){//由于浮点数精度问题，为避免调试时Ki由0.某个数值造成积分累加过多导致饱和，可加入该语句
//   errorint = 0;
// }else{
//   errorint += error0;
// }
// /*进行PID运算*/
// out = Kp*error0 + Ki*errorint + Kd*(error0 - error1);
// /*输出限幅*/
// if(out > 100) out = 100;
// else if(out < -100) out = -100;
// /*执行控制*/
// TB6612_Control(MOTOR_Left, out);

///*模板PID：增量式PID（带控制器内积分）实现电机定速控制*/
// /*获取实际值*/
// actual = ENCODER_GetPulse(ENCODER_Left);
// /*计算中间变量*/
// error2 = error1;
// error1 = error0;
// error0 = target - actual;
// /*进行PID运算*/
// out += Kp * (error0-error1) + Ki * error0 + Kd * (error0-2*error1+error2);
// /*输出限幅*/
// if(out > 100) out = 100;
// else if(out < -100) out = -100;
// /*执行控制*/
// TB6612_Control(MOTOR_Left, out);

///*模板PID：位置式PID实现电机定位置控制*/
// /*获取实际值*/
// actual += ENCODER_GetPulse(ENCODER_Left);
// /*计算中间变量*/
// error1 = error0;
// error0 = target - actual;
// // errorint += error0;
// if(fabs(Ki) < 0.0001){//由于浮点数精度问题，为避免调试时Ki由0.某个数值造成积分累加过多导致饱和，可加入该语句
//   errorint = 0;
// }else{
//   errorint += error0;
// }
// /*进行PID运算*/
// out = Kp*error0 + Ki*errorint + Kd*(error0 - error1);
// /*输出限幅*/
// if(out > 100) out = 100;
// else if(out < -100) out = -100;
// /*执行控制*/
// TB6612_Control(MOTOR_Left, out);

///*模板PID：增量式PID（带控制器内积分）实现电机定位置控制*/
// /*获取实际值*/
// actual += ENCODER_GetPulse(ENCODER_Left);
// /*计算中间变量*/
// error2 = error1;
// error1 = error0;
// error0 = target - actual;
// /*进行PID运算*/
// out += Kp * (error0-error1) + Ki * error0 + Kd * (error0-2*error1+error2);
// /*输出限幅*/
// if(out > 100) out = 100;
// else if(out < -100) out = -100;
// /*执行控制*/
// TB6612_Control(MOTOR_Left, out);

// /*PID算法改进措施：积分限幅--应用于位置式PID的积分项，解决误差长时间存在导致的误差积分进入深度饱和状态问题*/
// //此处的上下限可由 (out/Ki)得到
// if(errorint > 上限) errorint = 上限;
// else if(errorint < 下限) errorint = 下限;

// /*PID算法改进措施：积分分离--应用于位置式PID的积分项，解决在无稳态误差的系统中前期积分过大导致的积分超调问题*/
// //此处的阈值需要实测目标值和实际值相差比较小时的误差得到
// if(fabs(error0) < 阈值){
//   errorint += error0;
// }else{
//   errorint = 0;
// }

// /*PID算法改进措施：变速积分--应用于位置式PID的积分项，解决积分分离阈值没设对使得实际值刚好停在阈值外导致的没有积分效果问题*/
// //变速积分需要设计一个函数，随着本次误差绝对值的减小而增大函数值（调整系数），可以有线性、非线性等多种函数方案。调整系数可用于调整误差积分速度或者积分项作用强度。
// //此处以y=1/(k*fabs(本次误差)+1)函数 配合 调整系数*误差积分为例，k用于决定衰减速度。
// float C = 1 / (k*fabs(error0)+1);
// errorint += C * error0;

// /*PID算法改进措施：微分先行--应用于位置式PID的微分项，解决目标值大幅跳变时误差微分计算的微分项在目标值切换瞬间导致的输出一个很大的正向尖峰问题*/
// actual1 = actual;
// actual = ENCODER_GetPulse(ENCODER_Left);
// difout = -Kd * (actual - actual1);

// /*PID算法改进措施：不完全微分--应用于位置式PID的微分项，解决输入实际值存在噪声导致的微分项输出抖动问题*/
// //此处的a为滤波强度，范围0.0~1.0
// difout = (1-a) * Kd * (error0-error1) + a * difout;

// /*PID算法改进措施：输出偏移--应用于位置式PID的输出，解决输出值太小使得执行器不发生动作导致的调试误差问题*/
// if(out > 0.1){//由于浮点数精度问题，留一些余量
//   out += offset1;
// }else if(OUT < -0.1){
//   out -= offset2;
// }else{
//   out = 0;
// }

// /*PID算法改进措施：输入死区--应用于位置式PID的输入，解决实际值或目标值有细微噪声波动或系统有一定滞后导致的执行器在误差很小时不断进行调控问题*/
// if(fabs(error0) < 死区阈值){
//   out = 0;
// }else{
//   //PID运算 此处可放误差积分也可不放
// }

// /*双环PID：电机定速定位置控制--内环速度环，外环位置环*/
// //内环（调控周期要小于等于外环）
// speed = ENCODER_GetPulse(ENCODER_Left);//更新实际值
// location += speed;
// inner.actual = speed; //获取实际值
// PID_PositionUpdate(&inner);//进行PID运算
// TB6612_Control(MOTOR_Left, outer.out);//利用输出值控制执行器运动
// //外环
// outer.actual = location;  //获取实际值
// PID_PositionUpdate(&outer);//进行PID运算
// inner.target = outer.out;//传递输出值给下一级PID






