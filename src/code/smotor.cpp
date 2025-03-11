#include "smotor.h"
#include "pid.h"


void smotor_control(unsigned short duty) {
    //pwm_duty(SMOTOR_PIN, (unsigned short) duty);
}

// 角度转对应占空比
unsigned short servo_duty(double angle) {
    return (angle * 2 / 180 + 0.5) * 10000 * SERVO_FREQ / 1000;
}