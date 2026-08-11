#ifndef _velocityProfile_h
#define _velocityProfile_h

//#include "zf_common_headfile.h"

/*****************************************************************//**
 * \file   velocityProfile.h
 * \brief  梯形加减速算法模块相关结构体及函数声明。
 * 
 * \author galaxy
 * \date   March 2023
 *********************************************************************/



 /**
  * \brief 梯形速度曲线结构体.速度规划完成后，对应的关键参数以及一些辅助变量
  * 都会计算出来，避免后面插补模块重复计算。
  */
typedef struct {
	double L;//路程总长度
	double t1;//加速度段时间
	double t2;//匀速段时间
	double t3;//减速段时间
	double t; //总时间
	double L1;//加速段位移
	double L2;//匀速段位移
	double L3;//减速段位移
	double vs;//起步速度
	double vc;//匀速段期望速度
	double ve;//终止速度
	double acc;//加速度
	double dec;//减速度
}TrapeVelprofile_t;

/**
 * \brief 梯形加减速规划.
 *
 * \param tp	梯形速度曲线结构体。
 * \param L	位移。
 * \param vs	初始速度。
 * \param vc	期望速度。
 * \param ve	末速度。
 * \param acc	加速度。
 * \param dec   减速度。
 * \return      返回0执行成功，非零值执行失败。
 */
int calcTrapezoidalProfile(double L, double vs, double vmax, double ve, double amax, double dmax, TrapeVelprofile_t* tp);

/**
 * \brief 计算梯形加减速任意时刻的加速度.
 *
 * \param tp	已规划好的梯形速度结构体。
 * \param t		相对于梯形曲线起点的时刻。
 * \return		对应时刻的加速度。
 */
double calcTrapezoidalAcc(TrapeVelprofile_t* tp, double t);

/**
 * \brief 计算梯形加减速任意时刻的速度.
 *
 * \param tp	梯形速度曲线结构体。
 * \param t		相对于该段起点的时刻。
 * \return		对应时刻的位移。
 */
double  calcTrapezoidalVel(TrapeVelprofile_t* tp, double t);

/**
 * \brief 计算梯形加减速任意时刻的位移.
 *
 * \param tp	梯形速度曲线结构体。
 * \param t		相对于该段起点的时刻。
 * \return		对应时刻的位移。
 */
double  calcTrapezoidalDist(TrapeVelprofile_t* tp, double t);

#endif
