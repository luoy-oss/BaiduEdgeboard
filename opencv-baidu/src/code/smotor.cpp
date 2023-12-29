#include "smotor.h"
#include "pid.h"

void smotor_init(void) {
    //pwm_init(SMOTOR1_PIN, SERVO_FREQ, servo_duty(SMOTOR1_CENTER));
    //pwm_init(SMOTOR2_PIN, SERVO_FREQ, servo_duty(SMOTOR2_CENTER));
    //pwm_init(SMOTOR3_PIN, SERVO_FREQ, servo_duty(SMOTOR3_CENTER));
}


void smotor1_control(short duty) {
    //pwm_duty(SMOTOR1_PIN, (short)duty);

}
//
////+左转, -右转
//void smotor2_control(short duty) {
//    pwm_duty(SMOTOR2_PIN, (short)MINMAX(duty, 1250, 6250));
//}
//
////+低头, -抬头
//void smotor3_control(short duty) {
//    pwm_duty(SMOTOR3_PIN, (short)MINMAX(duty, 3750, 4850));
//}

// 角度转对应占空比
short servo_duty(float angle) {
    return (angle * 2 / 180 + 0.5) * 50000/*PWM_DUTY_MAX 默认50000*/ * SERVO_FREQ / 1000;
}