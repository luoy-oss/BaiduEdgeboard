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
int garage_found_num = 0;

float angle_diff(float a1, float a2) {
    float c1 = cosf(a1 / 180 * PI);
    float s1 = sinf(a1 / 180 * PI);
    float c2 = cosf(a2 / 180 * PI);
    float s2 = sinf(a2 / 180 * PI);
    float c = c1 * c2 + s1 * s2;
    float s = s1 * c2 - s2 * c1;
    return atan2f(s, c) * 180 / PI;
}

int zebra_cross_flag[30];
int zebra_cross_flag_num = 0;

int garage_x_begin = 90;
int garage_x_end = 236;
int garage_y_begin = 170;
int garage_y_end = 170;

void check_garage() {
    // 非车库模式下识别车库
    if (garage_type == GARAGE_NONE || garage_type == GARAGE_FOUND) {
        {
            // 远斑马线减速

            int c_far = 110;
            int r_far = 68;

            int pt[2] = { c_far, r_far };
            memset(zebra_cross_flag, 0, sizeof(zebra_cross_flag));
            zebra_cross_flag_num = 0;
            for (c_far = 110; c_far <= 210; c_far++) {
                pt[0] = c_far;
                pt[1] = r_far;

                if (zebra_cross_flag_num % 2 == 0 && AT_IMAGE(&img_raw, c_far, pt[1]) > thres) { // current even, current white
                    zebra_cross_flag[++zebra_cross_flag_num]++;
                }
                else if (zebra_cross_flag_num % 2 == 0 && AT_IMAGE(&img_raw, c_far, pt[1]) < thres) { // current even, current black
                    zebra_cross_flag[zebra_cross_flag_num]++;
                }
                else if (zebra_cross_flag_num % 2 == 1 && AT_IMAGE(&img_raw, c_far, pt[1]) > thres) { // current odd, current white
                    zebra_cross_flag[zebra_cross_flag_num]++;
                }
                else if (zebra_cross_flag_num % 2 == 1 && AT_IMAGE(&img_raw, c_far, pt[1]) < thres) { // current odd, current black
                    zebra_cross_flag[++zebra_cross_flag_num]++;
                }


                // 判断连续跳变的阈值条件以识别斑马线
                int i = 1;
                for (; i < zebra_cross_flag_num - 1; i++) {
                    if (zebra_cross_flag[i] < 2 || zebra_cross_flag[i] >= 20 || abs(zebra_cross_flag[i + 1] - zebra_cross_flag[i]) >= 10) break;
                }
                bool is_zebra = i > 6;

                if (is_zebra) {
                    garage_type = GARAGE_FOUND;
                    break;
                }
            }
        }

        // 近斑马线判断
        int c = 90;
        int r = 170;

        int pt[2] = { c, r };
        memset(zebra_cross_flag, 0, sizeof(zebra_cross_flag));
        zebra_cross_flag_num = 0;
        for (c = 90; c <= 236; c++) {
            pt[0] = c;
            pt[1] = r;

            if (zebra_cross_flag_num % 2 == 0 && AT_IMAGE(&img_raw, c, pt[1]) > thres) { // current even, current white
                zebra_cross_flag[++zebra_cross_flag_num]++;
            }
            else if (zebra_cross_flag_num % 2 == 0 && AT_IMAGE(&img_raw, c, pt[1]) < thres) { // current even, current black
                zebra_cross_flag[zebra_cross_flag_num]++;
            }
            else if (zebra_cross_flag_num % 2 == 1 && AT_IMAGE(&img_raw, c, pt[1]) > thres) { // current odd, current white
                zebra_cross_flag[zebra_cross_flag_num]++;
            }
            else if (zebra_cross_flag_num % 2 == 1 && AT_IMAGE(&img_raw, c, pt[1]) < thres) { // current odd, current black
                zebra_cross_flag[++zebra_cross_flag_num]++;
            }
            
            assert(zebra_cross_flag_num <= 29);
            // 判断连续跳变的阈值条件以识别斑马线
            int i = 1;
            for (; i < zebra_cross_flag_num - 1; i++) {
                if (zebra_cross_flag[i] < 2 || zebra_cross_flag[i] >= 20 || abs(zebra_cross_flag[i + 1] - zebra_cross_flag[i]) >= 10) break;
            }
            bool is_zebra = i > 6;

            if (is_zebra) {
                COUT1("is_zebra");
                if (++garage_num >= 2) {    // 第二次车库就入库
                    COUT1("第二次车库就入库");
                    garage_num = 0;
                    garage_type = GARAGE_IN;
                }
                else {                    // 第一次车库就不入库
                    COUT1("第一次车库就不入库");
                    garage_num = 0;
                    garage_type = GARAGE_PASS;
                }
                break;
            }
        }
    }
}

void run_garage() {
    switch (garage_type) {
    case GARAGE_OUT:
        // aim_distance = 0.25;
        track_type = TRACK_LEFT;
        break;
    case GARAGE_IN:
        // aim_distance = 0.5;
        track_type = TRACK_LEFT;
        stop = true;
        break;
    case GARAGE_PASS:
        track_type = TRACK_RIGHT;
        // 不入库，通过编码器使得小车走过车库后才退出车库模式
        if (garage_num++ > 30) {
            garage_type = GARAGE_NONE;
        }
        break;
    case GARAGE_FOUND:
        COUT1(">>>>>>>>>>发现远处车库，执行减速！！！！！");
        if (garage_found_num++ > 30) {
            garage_found_num = 0;
            garage_type = GARAGE_NONE;
        }
        break;
    default:
        (void)0;
    }
}
