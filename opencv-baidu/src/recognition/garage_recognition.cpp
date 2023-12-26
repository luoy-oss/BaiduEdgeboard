#include "garage_recognition.h"
#include "../code/camera_param.h"
#include "../src/main.h"

enum garage_type_e garage_type = GARAGE_NONE;// GARAGE_OUT;

const char* garage_type_name[GARAGE_NUM] = {
        "GARAGE_NONE",
        "GARAGE_OUT",
        "GARAGE_FOUND",
        "GARAGE_IN",
        "GARAGE_PASS",
        "GARAGE_STOP",
};

// 记录当前第几次车库
int garage_num = 0;

float angle_diff(float a1, float a2) {
    float c1 = cosf(a1 / 180 * PI);
    float s1 = sinf(a1 / 180 * PI);
    float c2 = cosf(a2 / 180 * PI);
    float s2 = sinf(a2 / 180 * PI);
    float c = c1 * c2 + s1 * s2;
    float s = s1 * c2 - s2 * c1;
    return atan2f(s, c) * 180 / PI;
}

int zebra_cross_flag_begin = 0;
int zebra_cross_flag0[30];
int zebra_cross_flag0_num = 0;
int zebra_cross_flag1[30];
int zebra_cross_flag1_num = 0;

float(*garage_rpts)[2];
int garage_rpts_num;

void check_garage() {

}

void run_garage() {

}
