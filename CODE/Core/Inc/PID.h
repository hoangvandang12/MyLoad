#ifndef _PID_H_
#define _PID_H_

#include "main.h"
#include "stdbool.h"

//#define Kp 230.0
//#define Ki 0.5
//#define Kd 155.0
//#define dt 0.1

//uint16_t PID(float point, float set_point, uint16_t max, uint16_t min);

uint16_t PID_VR2(float point, float set_point, uint16_t max, uint16_t min, bool inver, float Kp, float Ki, float Kd, float T, bool rst);

#endif
