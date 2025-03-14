#include "pid.h"

// 常规PID
double pid_solve(pid_param_t* pid, double error) {
    pid->out_d = (error - pid->out_p) * pid->low_pass + pid->out_d * (1 - pid->low_pass);
    pid->out_p = error;
    pid->out_i += error;

    if (pid->ki != 0) pid->out_i = MINMAX(pid->out_i, -pid->i_max / pid->ki, pid->i_max / pid->ki);
    
    return pid->kp * pid->out_p + pid->ki * pid->out_i + pid->kd * pid->out_d;
}

// 前馈pid
double ff_pid_solve(pid_param_t* pid, double error) {
    pid->out_d = (error - pid->out_p) * pid->low_pass + pid->out_d * (1 - pid->low_pass);
    pid->out_p = error;
    pid->out_i += error;

    if (pid->ki != 0) pid->out_i = MINMAX(pid->out_i, -pid->i_max / pid->ki, pid->i_max / pid->ki);
    
    return pid->kp * pid->out_p + pid->ki * pid->out_i + pid->kd * pid->out_d + pid->kf * error;
}

// 快速pid - 2024年4月23日
double pid_2_solve(pid_param_t* pid, double error) {
    pid->out_d = error - pid->out_p;
    pid->out_p = error;
    
    return pid->kp * pid->out_p + pid->kp2 * abs(pid->out_p) * pid->out_p + pid->kd * pid->out_d;
}

// 增量式PID
double increment_pid_solve(pid_param_t* pid, double error) {
    pid->out_d = MINMAX(pid->kd * (error - 2 * pid->pre_error + pid->pre_pre_error), -pid->d_max, pid->d_max);
    pid->out_p = MINMAX(pid->kp * (error - pid->pre_error), -pid->p_max, pid->p_max);
    pid->out_i = MINMAX(pid->ki * error, -pid->i_max, pid->i_max);

    pid->pre_pre_error = pid->pre_error;
    pid->pre_error = error;

    return pid->out_p + pid->out_i + pid->out_d;
}


double change_kib = 4;

//变积分PID，e大i小
double changable_pid_solve(pid_param_t* pid, double error) {
    pid->out_d = pid->kd * (error - 2 * pid->pre_error + pid->pre_pre_error);
    pid->out_p = pid->kp * (error - pid->pre_error);
    double ki_index = pid->ki;
    if (error + pid->pre_error > 0) {
        ki_index = (pid->ki) - (pid->ki) / (1 + exp(change_kib - 2.9 * fabs(error)));    //变积分控制
    }

    pid->out_i = ki_index * error;
    pid->pre_pre_error = pid->pre_error;
    pid->pre_error = error;

    return MINMAX(pid->out_p, -pid->p_max, pid->p_max)
        + MINMAX(pid->out_i, -pid->i_max, pid->i_max)
        + MINMAX(pid->out_d, -pid->d_max, pid->d_max);
}

double bangbang_out = 0;

double bangbang_pid_solve(pid_param_t* pid, double error) {
    double BangBang_output = 15000, BangBang_error = 8;
    pid->error = error;

    //BangBang
    if (error > BangBang_error || error < -BangBang_error) {
        bangbang_out = (error > 0) ? BangBang_output : (-BangBang_output);
    }
    else {
        pid->out_d = pid->kd * (error - 2 * pid->pre_error + pid->pre_pre_error);

        pid->out_p = pid->kp * (error - pid->pre_error);

        pid->out_i = pid->ki * error;

        bangbang_out = MINMAX(pid->out_p, -pid->p_max, pid->p_max)
            + MINMAX(pid->out_i, -pid->i_max, pid->i_max)
            + MINMAX(pid->out_d, -pid->d_max, pid->d_max);

    }
    pid->pre_pre_error = pid->pre_error;
    pid->pre_error = error;

    return bangbang_out;
}


