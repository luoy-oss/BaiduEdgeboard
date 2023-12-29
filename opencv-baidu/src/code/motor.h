#ifndef _motor_h
#define _motor_h

#include "headfile.h"
#include "pid.h"


#define BYTE0(dwTemp) (*(char *)(&dwTemp))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

enum motor_mode_e {
    MODE_NORMAL, MODE_BANGBANG, MODE_SOFT, MODE_POSLOOP,
};

struct motor_param_t {
    float target_speed;
    int32_t duty;         //Motor PWM duty
    motor_mode_e motor_mode;

    pid_param_t pid;            //Motor PID param
    pid_param_t brake_pid;      //Motor PID param
    motor_param_t() {
        target_speed = 0;
        motor_mode = MODE_NORMAL;
        duty = 0;
    }

    motor_param_t(int ts, 
        int kp, int ki, int kd, 
        int brake_kp, int brake_ki, int brake_kd, 
        int low_pass, int p_max, int i_max, int d_max) {
        target_speed = ts;
        pid = pid_param_t(kp, ki, kd, low_pass, p_max, i_max, d_max);
        brake_pid = pid_param_t(brake_kp, brake_ki, brake_kd, low_pass, p_max, i_max, d_max);
        duty = 0;
    }

};


#define MOTOR_CREATE(ts, kp, ki, kd, brake_kp, brake_ki, brake_kd, low_pass, p_max, i_max, d_max)       \
    {                                           \
        .total_encoder = 0,                     \
        .encoder_speed = 0,                     \
        .target_speed = ts,                     \
        .motor_mode = MODE_NORMAL,              \
        .pid = PID_CREATE(kp, ki, kd, low_pass, p_max ,i_max ,d_max), \
        .brake_pid = PID_CREATE(brake_kp, brake_ki, brake_kd, low_pass, p_max ,i_max ,d_max), \
    }

extern motor_param_t motor_l, motor_r;


#define ENCODER_PER_METER   (5800)

void motor_control(void);

int speed_control();

#endif