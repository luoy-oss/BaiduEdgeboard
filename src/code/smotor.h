#ifndef _smotor_h
#define _smotor_h

#include "headfile.h"
#include "pid.h"

// 转向舵机朝向正前方时的角度
// #define SMOTOR_CENTER   (90)
#define SMOTOR_CENTER   (91.5)
#define STRAIGHT_SMOTOR_CENTER   (92.5)

#define SERVO_FREQ      (300)

// 前轮转角和方向舵机转角的比例关系
#define SMOTOR_RATE     (2.4)

extern pid_param_t servo_pid;

void smotor_control(unsigned short duty);

unsigned short servo_duty(double angle);

#endif
