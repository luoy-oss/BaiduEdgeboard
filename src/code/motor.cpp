#include "../recognition/cross_recognition.h"
#include "../recognition/circle_recognition.h"
#include "../recognition/garage_recognition.h"
#include "smotor.h"
#include "motor.h"
#include "../src/main.h"
// left
#define MOTOR1_PWM1     PWM2_MODULE3_CHB_D3
#define MOTOR1_PWM2     PWM1_MODULE3_CHB_D1
// right
#define MOTOR2_PWM1     PWM2_MODULE3_CHA_D2
#define MOTOR2_PWM2     PWM1_MODULE3_CHA_D0


//#define MINMAX(x, l, u) MIN(MAX(x, l), u)
#define MOTOR_PWM_DUTY_MAX    50000

//变积分PID
//motor_param_t motor_r = MOTOR_CREATE(12, 18, 1, 15, 2500, 250, 10,MOTOR_PWM_DUTY_MAX/3 ,MOTOR_PWM_DUTY_MAX/3 ,MOTOR_PWM_DUTY_MAX/3);

//常规增量PID
motor_param_t motor(12, 1000, 25, 100, 5000, 500, 600, 1, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
//motor_param_t motor_r(12, 1000, 25, 100, 5000, 500, 600, 1, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);


// Matlab位置PID
// 0.5s
pid_param_t motor_pid(7021, 10000 / 1e3, 0, 1, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
//pid_param_t motor_pid_r(7021, 10000 / 1e3, 0, 1, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
// 0.4s
//pid_param_t motor_pid = PID_CREATE(8830, 14117/1e3, 0, 1, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
//pid_param_t motor_pid_r = PID_CREATE(8830, 14117/1e3, 0, 1, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
// 0.3s
//pid_param_t motor_pid = PID_CREATE(11864, 22367/1e3, 0, 1, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
//pid_param_t motor_pid_r = PID_CREATE(11864, 22367/1e3, 0, 1, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);

// 从赛道弯曲情况，计算目标速度
pid_param_t target_speed_pid(5., 0, 30., 0.6, 5, 5, 5);

// 位置环PID
pid_param_t posloop_pid(200., 0, 0., 0.7, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);

// 基准速度
double NORMAL_SPEED = 25.;  //16.4
// 当前目标速度
double target_speed = NORMAL_SPEED;

//圆环速度 + NORMAL_SPEED
double CIRCLE_MAX_SPEED = 0, CIRCLE_MIN_SPEED = -4;
//速度限+  NORMAL_SPEED
double NORMAL_MAX_SPEED = 5, NORMAL_MIN_SPEED = -7;



// // 三点圆弧半径
// double radius_3pts(double pt0[2], double pt1[2], double pt2[2]) {
//     double a, b, c, d, e, f, r, x, y;
//     a = 2 * (pt1[0] - pt0[0]);
//     b = 2 * (pt1[1] - pt0[1]);
//     c = pt1[0] * pt1[0] + pt1[1] * pt1[1] - pt0[0] * pt0[0] - pt0[1] * pt0[1];
//     d = 2 * (pt2[0] - pt1[0]);
//     e = 2 * (pt2[1] - pt1[1]);
//     f = pt2[0] * pt2[0] + pt2[1] * pt2[1] - pt1[0] * pt1[0] - pt1[1] * pt1[1];
//     x = (b * f - e * c) / (b * d - e * a);
//     y = (d * c - a * f) / (b * d - e * a);
//     r = sqrt((x - pt0[0]) * (x - pt0[0]) + (y - pt0[1]) * (y - pt0[1]));
//     return r;
// }



int speed_control() {
    
    // 默认常规控制模式
    motor.motor_mode = MODE_NORMAL;
  
    if (garage_type == GARAGE_OUT) {
        // 出库缓启动模式，避免一下冲的太猛，冲出赛道
        motor.motor_mode = MODE_SOFT;
        target_speed = 14;
    }
    else if (garage_type == GARAGE_IN) {
        // 入库中减速
        target_speed = 10;
    }
    //else if (enable_adc) {
    //    // 坡道减速，避免飞坡翻车
    //    target_speed = 9;
    //    motor.motor_mode = MODE_BANGBANG;
    //    motor_r.motor_mode = MODE_BANGBANG;
    //}
    else if (circle_type == CIRCLE_LEFT_BEGIN || circle_type == CIRCLE_RIGHT_BEGIN) {
        // 圆环速度  左圆环max 16.2 -1.5
        // 圆环开始，斜坡函数减速
        target_speed = MINMAX(target_speed - 0.02, NORMAL_SPEED + CIRCLE_MIN_SPEED, NORMAL_SPEED + CIRCLE_MAX_SPEED);
    }
    //else if (circle_type == CIRCLE_END || circle_type == CIRCLE_RIGHT_END) {
    //    // 出环加速
    //    target_speed = MINMAX(target_speed + 0.01, NORMAL_SPEED + CIRCLE_MIN_SPEED, NORMAL_SPEED);
    //}
    else if (rptsn_num > 20) {
//         // 直道/弯道速度
//         int id = MIN(70, rptsn_num - 1);
//         double error = fabs((rptsn[id][0] - rptsn[0][0]) / (rptsn[id][1] - rptsn[0][1]));
//         // 减速加入kd, 突入大弯
// //        if(error >= 0.5) target_speed_pid.kd = 20;
// //        else target_speed_pid.kd = 0;

//         double speed = -pid_solve(&target_speed_pid, error);
//         //COUT2("error" + std::to_string(error), "speed" + std::to_string(speed));

//         target_speed = MINMAX(NORMAL_SPEED + speed, NORMAL_SPEED + NORMAL_MIN_SPEED, NORMAL_SPEED + NORMAL_MAX_SPEED);
        // 直道/弯道速度
        int id1 = MIN(40, rptsn_num - 1);
        int id2 = MIN(60, rptsn_num - 1);
        int id3 = MIN(80, rptsn_num - 1);
        double rad = radius_3pts(rptsn[id1], rptsn[id2], rptsn[id3]);
        if (rad < 100) {
            rad = log10(rad) / 2;
            target_speed = NORMAL_SPEED + (NORMAL_MIN_SPEED) * rad;
        }
        else {
            rad = (log10(rad) - 2)/1.7;
            target_speed = NORMAL_SPEED + (NORMAL_MAX_SPEED) * rad;
        }
    }
    else if (rptsn_num > 5) {
        //点太少,不对劲直接慢速
        target_speed = NORMAL_SPEED + NORMAL_MIN_SPEED;
        COUT1("点太少,不对劲直接慢速\ttarget_speed " + std::to_string(target_speed));
    }
    //// 急停(车库停车)
    //if (garage_type == GARAGE_STOP || (garage_type != GARAGE_OUT)) {
    //    motor.motor_mode = MODE_NORMAL;
    //    target_speed = 0;
    //    COUT1("急停(车库停车)\ttarget_speed 0");
    //}

    // 动态舵机Kp
    //servo_pid.kp = 1. + (motor.encoder_speed + motor_r.encoder_speed) / 40;

    // 附加差速
    return motor.target_speed = target_speed;
}


//void motor_control(void) {
//    //square_signal();
//
//    //wireless_show();
//
//    speed_control();
//
//
//    if (motor.motor_mode == MODE_NORMAL) {
//        // 常规模式
//        double error = (motor.target_speed - motor.encoder_speed);
//        motor.duty = pid_solve(&motor_pid, error);
//        motor.duty = MINMAX(motor.duty, -MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
//    }
//    else if (motor.motor_mode == MODE_BANGBANG) {
//        // 邦邦模式，即猛加猛减
//        motor_pid.out_i = 0;  // 清除默认模式的积分量
//
//        motor.duty += bangbang_pid_solve(&motor.brake_pid, (double)(motor.target_speed - motor.encoder_speed));
//        motor.duty = MINMAX(motor.duty, -MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
//    }
//    else if (motor.motor_mode == MODE_SOFT) {
//        // 缓启动模式
//        motor_pid.out_i = 0;  // 清除默认模式的积分量
//
//        motor.duty += changable_pid_solve(&motor.pid, (double)(motor.target_speed - motor.encoder_speed));
//        motor.duty = MINMAX(motor.duty, -MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
//    }
//    else if (motor.motor_mode == MODE_POSLOOP) {
//        //Apriltag停车位置环
//        motor_pid.out_i = 0; // 清除默认模式的积分量
//
//        motor.duty = pid_solve(&posloop_pid, (double)(motor.target_encoder - motor.total_encoder));
//        motor.duty = MINMAX(motor.duty, -MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
//    }
//
//    //大角度限幅
//    if (fabs(angle) > 10) {
//        //减速
//        if (target_speed - (motor_r.encoder_speed + motor.encoder_speed) / 2 < 0) {
//            motor.duty = MINMAX(motor.duty, -MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX * 8 / 10);
//            motor_r.duty = MINMAX(motor_r.duty, -MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX * 8 / 10);
//        }
//        else {
//            motor.duty = MINMAX(motor.duty, -MOTOR_PWM_DUTY_MAX * 8 / 10, MOTOR_PWM_DUTY_MAX * 8 / 10);
//            motor_r.duty = MINMAX(motor_r.duty, -MOTOR_PWM_DUTY_MAX * 8 / 10, MOTOR_PWM_DUTY_MAX * 8 / 10);
//        }
//    }
//
//
//    //PWM控制
//    pwm_duty(MOTOR1_PWM1, (motor.duty >= 0) ? motor.duty : 0);
//    pwm_duty(MOTOR1_PWM2, (motor.duty >= 0) ? 0 : (-motor.duty));
//
//    pwm_duty(MOTOR2_PWM1, (motor_r.duty >= 0) ? motor_r.duty : 0);
//    pwm_duty(MOTOR2_PWM2, (motor_r.duty >= 0) ? 0 : (-motor_r.duty));
//}
