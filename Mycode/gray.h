#ifndef __GRAY_H
#define __GRAY_H

#define GRAY1 1
#define GRAY2 2
#define GRAY3 3
#define GRAY4 4
#define GRAY5 5
#define GRAY6 6
#define GRAY7 7
#define GRAY8 8

extern uint8_t GRAY_Data[9];  //存储灰度传感器的值(下标为x 对应通道x)

GPIO_PinState  GRAY_ONE(uint8_t Channel);
void GRAY_ALL(void);

#endif
