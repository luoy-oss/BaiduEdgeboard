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
////+��ת, -��ת
//void smotor2_control(short duty) {
//    pwm_duty(SMOTOR2_PIN, (short)MINMAX(duty, 1250, 6250));
//}
//
////+��ͷ, -̧ͷ
//void smotor3_control(short duty) {
//    pwm_duty(SMOTOR3_PIN, (short)MINMAX(duty, 3750, 4850));
//}

// �Ƕ�ת��Ӧռ�ձ�
short servo_duty(float angle) {
    return (angle * 2 / 180 + 0.5) * 50000/*PWM_DUTY_MAX Ĭ��50000*/ * SERVO_FREQ / 1000;
}