#ifndef __MOTER_ZL_H
#define __MOTER_ZL_H

/*-------------------------------文件包含--------------------------------*/
#include "stm32f10x.h"
enum MOTER_DIR 
{
	FOR=0,//顺时针
	BAC,
	DIS,
	STO,
};
struct MOTER
{
	enum MOTER_DIR dir;
	float act_speed;
	float tar_speed;
	int act_location;
	int tar_location;
	int pwm_duty;
	
	
};
struct CAR
{
	struct MOTER moter1,moter2,moter3,moter4;
	char moter_en;//1为使能
	float act_angle;
	float tar_angle;
};
	
extern int speed_count;
extern struct CAR car;
void moter_init();
void speed_BasicTim_Init(void);
void cmd_data_recv(uint8_t *data, uint16_t data_len);
#endif





























