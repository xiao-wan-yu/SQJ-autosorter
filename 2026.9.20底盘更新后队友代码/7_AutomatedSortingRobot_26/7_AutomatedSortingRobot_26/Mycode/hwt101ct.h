#ifndef __HWT101CT_H
#define __HWT101CT_H

#include "stm32f4xx_hal.h"

typedef struct{
  float yaw;
}HWT101CT_DATA;

extern HWT101CT_DATA HWT101CT_Data;

void HWT101CT_Update(void);
void HWT101CT_Init(void);


#endif
