#ifndef _PID_H_
#define _PID_H_

#include "headfile.h"

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MINMAX(input, low, upper) MIN(MAX(input, low), upper)

typedef struct PID{
    float kp;    //P
    float ki;    //I
    float kd;    //D
    float i_max; //integrator_max
    float p_max; //integrator_max
    float d_max; //integrator_max

    float low_pass;

    float out_p;
    float out_i;
    float out_d;

    float error;
    float pre_error;
    float pre_pre_error;
    PID() {
        this->kp = 0.0f;
        this->ki = 0.0f;
        this->kd = 0.0f;
        this->low_pass = 0.0f;
        this->p_max = 0.0f;
        this->i_max = 0.0f;
        this->d_max = 0.0f;
        this->out_p = this->out_i = this->out_d = 0.0f;

        this->error = 0.0f;
        this->pre_error = 0.0f;
        this->pre_pre_error = 0.0f;
    }
    PID(const float _kp, const float _ki, const float _kd, const float _low_pass, const float max_p, const float max_i, const float max_d) {
        this->kp = _kp;
        this->ki = _ki;
        this->kd = _kd;
        this->low_pass = _low_pass;
        this->p_max = max_p;
        this->i_max = max_i;
        this->d_max = max_d;
        this->out_p = this->out_i = this->out_d = 0;
        
        this->error = 0.0f;
        this->pre_error = 0.0f;
        this->pre_pre_error = 0.0f;
    }
} pid_param_t;

#define PID_CREATE(_kp, _ki, _kd, _low_pass, max_p, max_i, max_d) \
    {                                    \
        .kp = _kp,                       \
        .ki = _ki,                       \
        .kd = _kd,                       \
        .low_pass = _low_pass,           \
        .out_p = 0,                      \
        .out_i = 0,                      \
        .out_d = 0,                      \
        .p_max = max_p,                  \
        .i_max = max_i,                  \
        .d_max = max_d,                  \
    }


float pid_solve(pid_param_t* pid, float error);

float increment_pid_solve(pid_param_t* pid, float error);

float bangbang_pid_solve(pid_param_t* pid, float error);

float changable_pid_solve(pid_param_t* pid, float error);

#endif /* _PID_H_ */
