#ifndef __SERVO_H
#define __SERVO_H

//舵机号码
#define SERVO1          1
#define SERVO2          2
#define SERVO3          3
#define SERVO4          4

//舵机号码对应的最大转动角度--根据实际情况选择90度/180度/270度
#define SERVO1_MaxAngle  180
#define SERVO2_MaxAngle  180
#define SERVO3_MaxAngle  180
#define SERVO4_MaxAngle  180

void SERVO_Control(uint8_t SERVO_Num, float SERVO_TargetAngle);

#endif
