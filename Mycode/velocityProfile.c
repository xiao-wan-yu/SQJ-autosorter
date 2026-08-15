#include <math.h>
#include <string.h>
#include <float.h>
#include "velocityProfile.h"

/**
 * \brief 梯形加减速规划.
 */
int calcTrapezoidalProfile(double L, double vs, double vmax, double ve, double amax, double dmax, TrapeVelprofile_t* tp)
{
	memset(tp, 0, sizeof(TrapeVelprofile_t));
	double vf = 0.0, vc = vmax, acc = amax, dec = -dmax;
	tp->acc = acc;
	tp->dec = dec;
	tp->L = L;
	/* 初速度和末速度不能大于最大速度 */
	if(vs > vc) vs = vc;
	if(ve > vc) ve = vc;
	if(L < 1.0E-5) return 0;
	/* 4段情况 */
	vf = sqrt((-2.0*acc*dec*L - vs*vs*dec + ve*ve*acc) / (acc - dec));
	if(vf > vc)  /* 达到最大速度 */
	{
		tp->vs = vs; tp->vc = vc; tp->ve = ve;
		tp->t1 = (vc - vs) / acc;
		tp->t3 = (ve - vc) / dec;
		tp->L1 = vs*tp->t1 + 0.5*acc*tp->t1*tp->t1;
		tp->L3 = vc*tp->t3 + 0.5*dec*tp->t3*tp->t3;
		tp->L2 = L - tp->L1 - tp->L3;
		tp->t2 = tp->L2 / vc;
		tp->t  = tp->t1 + tp->t2 + tp->t3;
	}
	else  /* 达不到最大速度 */
	{
		if(vs < vf && vf < ve)  /* 只有加速段 */
		{
			ve = sqrt(vs*vs + 2*acc*L);
			vc = ve;
			tp->vs = vs; tp->vc = vc; tp->ve = ve;
			tp->t1 = (ve - vs) / acc;
			tp->t2 = 0; tp->t3 = 0;
			tp->L1 = vs*tp->t1 + 0.5*acc*tp->t1*tp->t1;
			tp->L2 = 0; tp->L3 = 0;
			tp->t = tp->t1;
		}
		else if(ve < vf && vf < vs)  /* 只有减速段 */
		{
			ve = sqrt(vs*vs + 2*dec*L);
			vc = vs;
			tp->vs = vs; tp->vc = vs; tp->ve = ve;
			tp->t1 = 0; tp->t2 = 0;
			tp->t3 = (ve - vs) / dec;
			tp->L1 = 0; tp->L2 = 0;
			tp->L3 = vc*tp->t3 + 0.5*dec*tp->t3*tp->t3;
			tp->t = tp->t3;
		}
		else  /* 既有加速段也有减速段 */
		{
			vc = vf;
			tp->vs = vs; tp->vc = vc; tp->ve = ve;
			tp->t1 = (vc - vs) / acc;
			tp->t2 = 0;
			tp->t3 = (ve - vc) / dec;
			tp->L1 = vs*tp->t1 + 0.5*acc*tp->t1*tp->t1;
			tp->L2 = 0;
			tp->L3 = vc*tp->t3 + 0.5*dec*tp->t3*tp->t3;
			tp->t = tp->t1 + tp->t3;
		}
	}
	return 0;
}

double calcTrapezoidalAcc(TrapeVelprofile_t* tp, double t)
{
	if(t < 0.0)              return 0.0;
	else if(t < tp->t1)      return tp->acc;
	else if(t < tp->t1 + tp->t2) return 0.0;
	else                     return tp->dec;
}

double calcTrapezoidalVel(TrapeVelprofile_t* tp, double t)
{
	double vt = 0.0;
	if(tp->L < 1.0E-5) vt = 0.0;
	else if(t < tp->t1) vt = tp->vs + tp->acc * t;
	else if(t < tp->t1 + tp->t2) vt = tp->vc;
	else vt = tp->vc + tp->dec * (t - tp->t1 - tp->t2);
	return vt;
}

double calcTrapezoidalDist(TrapeVelprofile_t* tp, double t)
{
	double tmp = 0.0, Lt = 0.0;
	if(tp->L < 1.0E-5) Lt = 0.0;
	else if(t < tp->t1) Lt = tp->vs * t + 0.5 * tp->acc * t * t;
	else if(t < tp->t1 + tp->t2) Lt = tp->L1 + tp->vc * (t - tp->t1);
	else
	{
		tmp = t - tp->t1 - tp->t2;
		Lt = tp->L1 + tp->L2 + tp->vc * tmp + 0.5 * tp->dec * tmp * tmp;
	}
	return Lt;
}
