#ifndef _PID_H_
#define _PID_H_

#include "headfile.h"

// #define _MIN(a, b) (((a) < (b)) ? (a) : (b))
// #define _MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MINMAX(input, low, upper) MIN(MAX(input, low), upper)

typedef struct PID{
    double kp;    //P
    double kp2;   //P
    double kf;    //前馈比例
    double ki;    //I
    double kd;    //D
    double i_max; //integrator_max
    double p_max; //integrator_max
    double d_max; //integrator_max

    double low_pass;

    double out_p;
    double out_i;
    double out_d;

    double error;
    double pre_error;
    double pre_pre_error;
    PID() {
        this->kp = 0.0f;
        this->kf = 0.0f;
        this->kp2 = 0.0f;
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
    PID(const double _kp, const double _ki, const double _kd, const double _low_pass, const double max_p, const double max_i, const double max_d) {
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

    PID(const double _kp, const double _ki, const double _kd, const double _kf, const double _low_pass, const double max_p, const double max_i, const double max_d) {
        this->kp = _kp;
        this->ki = _ki;
        this->kd = _kd;
        this->kf = _kf;
        this->low_pass = _low_pass;
        this->p_max = max_p;
        this->i_max = max_i;
        this->d_max = max_d;
        this->out_p = this->out_i = this->out_d = 0;
        
        this->error = 0.0f;
        this->pre_error = 0.0f;
        this->pre_pre_error = 0.0f;
    }

    PID(const double _kp, const double _kp2, const double _kd) {
        this->kp = _kp;
        this->kp2 = _kp2;

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


double pid_solve(pid_param_t* pid, double error);

double ff_pid_solve(pid_param_t* pid, double error);

double pid_2_solve(pid_param_t* pid, double error);

double increment_pid_solve(pid_param_t* pid, double error);

double bangbang_pid_solve(pid_param_t* pid, double error);

double changable_pid_solve(pid_param_t* pid, double error);

#endif /* _PID_H_ */
