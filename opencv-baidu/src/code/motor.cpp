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

//�����PID
//motor_param_t motor_r = MOTOR_CREATE(12, 18, 1, 15, 2500, 250, 10,MOTOR_PWM_DUTY_MAX/3 ,MOTOR_PWM_DUTY_MAX/3 ,MOTOR_PWM_DUTY_MAX/3);

//��������PID
motor_param_t motor(12, 1000, 25, 100, 5000, 500, 600, 1, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
//motor_param_t motor_r(12, 1000, 25, 100, 5000, 500, 600, 1, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);


// Matlabλ��PID
// 0.5s
pid_param_t motor_pid(7021, 10000 / 1e3, 0, 1, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
//pid_param_t motor_pid_r(7021, 10000 / 1e3, 0, 1, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
// 0.4s
//pid_param_t motor_pid = PID_CREATE(8830, 14117/1e3, 0, 1, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
//pid_param_t motor_pid_r = PID_CREATE(8830, 14117/1e3, 0, 1, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
// 0.3s
//pid_param_t motor_pid = PID_CREATE(11864, 22367/1e3, 0, 1, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
//pid_param_t motor_pid_r = PID_CREATE(11864, 22367/1e3, 0, 1, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);

// �������������������Ŀ���ٶ�
pid_param_t target_speed_pid(5., 0, 30., 0.6, 5, 5, 5);

// λ�û�PID
pid_param_t posloop_pid(200., 0, 0., 0.7, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);

// ��׼�ٶ�
float NORMAL_SPEED = 25.;  //16.4
// ��ǰĿ���ٶ�
float target_speed = NORMAL_SPEED;

//Բ���ٶ� + NORMAL_SPEED
float CIRCLE_MAX_SPEED = 0, CIRCLE_MIN_SPEED = -4;
//�ٶ���+  NORMAL_SPEED
float NORMAL_MAX_SPEED = 5, NORMAL_MIN_SPEED = -7;



// // ����Բ���뾶
// float radius_3pts(float pt0[2], float pt1[2], float pt2[2]) {
//     float a, b, c, d, e, f, r, x, y;
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

    // Ĭ�ϳ������ģʽ
    motor.motor_mode = MODE_NORMAL;

    if (garage_type == GARAGE_OUT) {
        // ���⻺����ģʽ������һ�³��̫�ͣ��������
        motor.motor_mode = MODE_SOFT;
        target_speed = 14;
    }
    else if (garage_type == GARAGE_IN) {
        // ����м���
        target_speed = 10;
    }
    //else if (enable_adc) {
    //    // �µ����٣�������·���
    //    target_speed = 9;
    //    motor.motor_mode = MODE_BANGBANG;
    //    motor_r.motor_mode = MODE_BANGBANG;
    //}
    else if (circle_type == CIRCLE_LEFT_BEGIN || circle_type == CIRCLE_RIGHT_BEGIN) {
        // Բ���ٶ�  ��Բ��max 16.2 -1.5
        // Բ����ʼ��б�º�������
        target_speed = MINMAX(target_speed - 0.02, NORMAL_SPEED + CIRCLE_MIN_SPEED, NORMAL_SPEED + CIRCLE_MAX_SPEED);
    }
    //else if (circle_type == CIRCLE_END || circle_type == CIRCLE_RIGHT_END) {
    //    // ��������
    //    target_speed = MINMAX(target_speed + 0.01, NORMAL_SPEED + CIRCLE_MIN_SPEED, NORMAL_SPEED);
    //}
    else if (rptsn_num > 20) {
        //         // ֱ��/����ٶ�
        //         int id = MIN(70, rptsn_num - 1);
        //         float error = fabs((rptsn[id][0] - rptsn[0][0]) / (rptsn[id][1] - rptsn[0][1]));
        //         // ���ټ���kd, ͻ�����
        // //        if(error >= 0.5) target_speed_pid.kd = 20;
        // //        else target_speed_pid.kd = 0;

        //         float speed = -pid_solve(&target_speed_pid, error);
        //         //COUT2("error" + std::to_string(error), "speed" + std::to_string(speed));

        //         target_speed = MINMAX(NORMAL_SPEED + speed, NORMAL_SPEED + NORMAL_MIN_SPEED, NORMAL_SPEED + NORMAL_MAX_SPEED);
                // ֱ��/����ٶ�
        int id1 = MIN(40, rptsn_num - 1);
        int id2 = MIN(60, rptsn_num - 1);
        int id3 = MIN(80, rptsn_num - 1);
        float rad = radius_3pts(rptsn[id1], rptsn[id2], rptsn[id3]);
        if (rad < 100) {
            rad = log10(rad) / 2;
            target_speed = NORMAL_SPEED + (NORMAL_MIN_SPEED)*rad;
        }
        else {
            rad = (log10(rad) - 2) / 1.7;
            target_speed = NORMAL_SPEED + (NORMAL_MAX_SPEED)*rad;
        }
    }
    else if (rptsn_num > 5) {
        //��̫��,���Ծ�ֱ������
        target_speed = NORMAL_SPEED + NORMAL_MIN_SPEED;
        COUT1("��̫��,���Ծ�ֱ������\ttarget_speed " + std::to_string(target_speed));
    }
    //// ��ͣ(����ͣ��)
    //if (garage_type == GARAGE_STOP || (garage_type != GARAGE_OUT)) {
    //    motor.motor_mode = MODE_NORMAL;
    //    target_speed = 0;
    //    COUT1("��ͣ(����ͣ��)\ttarget_speed 0");
    //}

    // ��̬���Kp
    //servo_pid.kp = 1. + (motor.encoder_speed + motor_r.encoder_speed) / 40;

    // ���Ӳ���
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
//        // ����ģʽ
//        float error = (motor.target_speed - motor.encoder_speed);
//        motor.duty = pid_solve(&motor_pid, error);
//        motor.duty = MINMAX(motor.duty, -MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
//    }
//    else if (motor.motor_mode == MODE_BANGBANG) {
//        // ���ģʽ�����ͼ��ͼ�
//        motor_pid.out_i = 0;  // ���Ĭ��ģʽ�Ļ�����
//
//        motor.duty += bangbang_pid_solve(&motor.brake_pid, (float)(motor.target_speed - motor.encoder_speed));
//        motor.duty = MINMAX(motor.duty, -MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
//    }
//    else if (motor.motor_mode == MODE_SOFT) {
//        // ������ģʽ
//        motor_pid.out_i = 0;  // ���Ĭ��ģʽ�Ļ�����
//
//        motor.duty += changable_pid_solve(&motor.pid, (float)(motor.target_speed - motor.encoder_speed));
//        motor.duty = MINMAX(motor.duty, -MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
//    }
//    else if (motor.motor_mode == MODE_POSLOOP) {
//        //Apriltagͣ��λ�û�
//        motor_pid.out_i = 0; // ���Ĭ��ģʽ�Ļ�����
//
//        motor.duty = pid_solve(&posloop_pid, (float)(motor.target_encoder - motor.total_encoder));
//        motor.duty = MINMAX(motor.duty, -MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
//    }
//
//    //��Ƕ��޷�
//    if (fabs(angle) > 10) {
//        //����
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
//    //PWM����
//    pwm_duty(MOTOR1_PWM1, (motor.duty >= 0) ? motor.duty : 0);
//    pwm_duty(MOTOR1_PWM2, (motor.duty >= 0) ? 0 : (-motor.duty));
//
//    pwm_duty(MOTOR2_PWM1, (motor_r.duty >= 0) ? motor_r.duty : 0);
//    pwm_duty(MOTOR2_PWM2, (motor_r.duty >= 0) ? 0 : (-motor_r.duty));
//}
