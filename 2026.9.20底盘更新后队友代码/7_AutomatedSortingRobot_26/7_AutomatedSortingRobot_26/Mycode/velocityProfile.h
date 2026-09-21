#ifndef _VELOCITY_PROFILE_H
#define _VELOCITY_PROFILE_H

/* 梯形速度规划 (移植自去年代码) */

typedef struct {
	double L;   // 路径总长
	double t1;  // 加速段时间
	double t2;  // 匀速段时间
	double t3;  // 减速段时间
	double t;   // 总时间
	double L1;  // 加速段位移
	double L2;  // 匀速段位移
	double L3;  // 减速段位移
	double vs;  // 初速度
	double vc;  // 匀速速度(最大速度)
	double ve;  // 末速度
	double acc; // 加速度
	double dec; // 减速度(负值)
} TrapeVelprofile_t;

int    calcTrapezoidalProfile(double L, double vs, double vmax, double ve, double amax, double dmax, TrapeVelprofile_t* tp);
double calcTrapezoidalAcc(TrapeVelprofile_t* tp, double t);
double calcTrapezoidalVel(TrapeVelprofile_t* tp, double t);
double calcTrapezoidalDist(TrapeVelprofile_t* tp, double t);

#endif /* _VELOCITY_PROFILE_H */
