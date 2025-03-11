#include "circle_recognition.h"
#include "../code/camera_param.h"
#include "../src/main.h"
#include <vector>
#include <numeric>
#include <cmath>

using namespace std;
enum circle_type_e circle_type = CIRCLE_NONE;

//方便串口收发
const char* circle_type_name[CIRCLE_NUM] = {
        "CIRCLE_NONE",
        "CIRCLE_LEFT_BEGIN", "CIRCLE_RIGHT_BEGIN",
        "CIRCLE_LEFT_IN", "CIRCLE_RIGHT_IN",
        "CIRCLE_LEFT_RUNNING", "CIRCLE_RIGHT_RUNNING",
        "CIRCLE_LEFT_OUT", "CIRCLE_RIGHT_OUT",
        "CIRCLE_LEFT_END", "CIRCLE_RIGHT_END",
};

// 入环上拐点逆透视行数虚大于该值(值越小入的越早)
// const int LEFT_IN_UPLPY = 140;
// const int RIGHT_IN_UPLPY = 140;
const int LEFT_IN_UPLPY = 140;
const int RIGHT_IN_UPLPY = 140;

bool OUT_NONE_LINE = false;

//// 编码器，用于防止一些重复触发等。
//int64_t circle_encoder;

int none_left_line = 0, none_right_line = 0;
int have_left_line = 0, have_right_line = 0;

// 原图左右边线
int circle_ipts[POINTS_MAX_LEN][2];
int circle_ipts_num;
// 变换后左右边线
float circle_rpts[POINTS_MAX_LEN][2];
int circle_rpts_num;
// 变换后左右边线+滤波
float circle_rptsb[POINTS_MAX_LEN][2];
int circle_rptsb_num;
// 变换后左右边线+等距采样
float circle_rptss[POINTS_MAX_LEN][2];
int circle_rptss_num;
// 左右边线局部角度变化率
float circle_rptsa[POINTS_MAX_LEN];
int circle_rptsa_num;
// 左右边线局部角度变化率+非极大抑制
float circle_rptsan[POINTS_MAX_LEN];
int circle_rptsan_num;

// L角点
int circle_Lpt_rptss_id = 0;
bool circle_Lpt_found = false;

int circle_count = 0;
const int CIRCLE_COUNT = 65;
float radius = 10000.0;

//寻找上拐点（辅助入环）
void find_upcorners() {
    // 识别Y,L拐点
    circle_Lpt_found = false;

    for (int i = 0; i < circle_rptss_num; i++) {
        if (circle_rptsan[i] == 0) continue;
        int im1 = clip(i - (int)round(angle_dist / sample_dist), 0, circle_rptss_num - 1);
        int ip1 = clip(i + (int)round(angle_dist / sample_dist), 0, circle_rptss_num - 1);
        float conf = fabs(circle_rptsan[i]) - (fabs(circle_rptsan[im1]) + fabs(circle_rptsan[ip1])) / 2;

        if (circle_Lpt_found == false && 70. / 180. * PI < conf && conf < 120. / 180. * PI) {
            circle_Lpt_rptss_id = i;
            circle_Lpt_found = true;
            break;
        }
    }

}

void circle_right_in_check() {
    // 原图找左右边线
    int top_id = 0;
    int x1 = 0, y1 = 0;

    if (ipts1_num > 50) {
        for (int i = 10; i < ipts1_num - 10; i++) {
            int x1 = ipts1[i - 10][0];
            int x2 = ipts1[i - 5][0];
            int x3 = ipts1[i][0];
            int x4 = ipts1[i + 5][0];
            int x5 = ipts1[i + 10][0];

            if (x3 <= x2 && x3 <= x4 && x3 <= x1 && x3 <= x5) {
                top_id = i;
                break;
            }
        }

    }
    else {
        top_id = 0;
    }
    if (ipts1_num > 10) {
        if (top_id) {
            x1 = clip(ipts1[top_id][0] + 5,  0, COLSIMAGE - 1);
            y1 = clip(ipts1[top_id][1] - 35, 0, ROWSIMAGE - 1);
        }
        else {
            x1 = clip(ipts1[top_id][0],  0, COLSIMAGE - 1);
            y1 = clip(ipts1[top_id][1] - 15, 0, ROWSIMAGE - 1);
        }
#ifdef CAR_DEBUG
        extern cv::Mat imageCorrect;
        cv::circle(imageCorrect, cv::Point(x1, y1), 5, cv::Scalar(255, 245, 0));

        draw_o(&img_line, clip((int)mapx[y1][x1], 0, img_line.width - 1),
            clip((int)mapy[y1][x1], 0, img_line.height - 1), 5, 255);
#endif
        circle_ipts_num = sizeof(circle_ipts) / sizeof(circle_ipts[0]);
        //for (; y1 > 0; y1--) if (AT_IMAGE(&img_raw, x1, y1 - 1) < thres) break;
        for (; y1 > 0; y1--) {
        
        #ifdef CAR_DEBUG
            MAT_AT_SET(imageCorrect, y1, x1, 255, 245, 0);
        
            AT_IMAGE(&img_line, clip((int)mapx[y1][x1], 0, img_line.width - 1),
                clip((int)mapy[y1][x1], 0, img_line.height - 1)) = 255;
        #endif

            if (AT_IMAGE(&img_raw, x1, y1 - 1) < thres) break;
        }
        if (AT_IMAGE(&img_raw, x1, y1) >= thres) {
            findline_righthand_adaptive(&img_raw, block_size, clip_value, x1, y1, circle_ipts, &circle_ipts_num);
        }
        else circle_ipts_num = 0;

        // 去畸变+透视变换（mapx，mapy，畸变坐标映射数组）
        for (int i = 0; i < circle_ipts_num; i++) {
            circle_rpts[i][0] = mapx[circle_ipts[i][1]][circle_ipts[i][0]];
            circle_rpts[i][1] = mapy[circle_ipts[i][1]][circle_ipts[i][0]];
        }
        circle_rpts_num = circle_ipts_num;

        // 边线滤波
        blur_points(circle_rpts, circle_rpts_num, circle_rptsb, (int)round(line_blur_kernel));
        circle_rptsb_num = circle_rpts_num;

        // 边线等距采样
        circle_rptss_num = sizeof(circle_rptss) / sizeof(circle_rptss[0]);
        resample_points(circle_rptsb, circle_rptsb_num, circle_rptss, &circle_rptss_num, 2 * sample_dist * pixel_per_meter);

        // 边线局部角度变化率
        local_angle_points(circle_rptss, circle_rptss_num, circle_rptsa, (int)round(angle_dist / sample_dist));
        circle_rptsa_num = circle_rptss_num;

        // 角度变化率非极大抑制
        nms_angle(circle_rptsa, circle_rptsa_num, circle_rptsan, (int)round(angle_dist / sample_dist) * 2 + 1);
        circle_rptsan_num = circle_rptsa_num;

    }
}


void circle_left_in_check() {
    // 原图找左右边线
    int top_id = ipts0_num - 10;
    int x1 = 0, y1 = 0;
    if (ipts0_num > 50) {
        for (int i = 10; i < ipts0_num - 10; i++) {
            int x1 = ipts0[i - 10][0];
            int x2 = ipts0[i - 5][0];
            int x3 = ipts0[i][0];
            int x4 = ipts0[i + 5][0];
            int x5 = ipts0[i + 10][0];

            if (x3 >= x2 && x3 >= x4 && x3 >= x1 && x3 >= x5) {
                top_id = i;
                break;
            }
        }
    }
    else {
        top_id = 0;
    }
    if (ipts0_num > 10) {
        if (top_id) {
            x1 = clip(ipts0[top_id][0] - 5,  0, COLSIMAGE - 1);
            y1 = clip(ipts0[top_id][1] - 35, 0, ROWSIMAGE - 1);
        }
        else {
            x1 = clip(ipts0[top_id][0],  0, COLSIMAGE - 1);
            y1 = clip(ipts0[top_id][1] - 15, 0, ROWSIMAGE - 1);
        }
#ifdef CAR_DEBUG
        extern cv::Mat imageCorrect;
        cv::circle(imageCorrect, cv::Point(x1, y1), 5, cv::Scalar(255, 245, 0));

        draw_o(&img_line, clip((int)mapx[y1][x1], 0, img_line.width - 1),
            clip((int)mapy[y1][x1], 0, img_line.height - 1), 5, 255);
#endif
        circle_ipts_num = sizeof(circle_ipts) / sizeof(circle_ipts[0]);
        //for (; y1 > 0; y1--) if (AT_IMAGE(&img_raw, x1, y1 - 1) < thres) break;
        for (; y1 > 0; y1--) {
#ifdef CAR_DEBUG
            MAT_AT_SET(imageCorrect, y1, x1, 255, 245, 0);

            AT_IMAGE(&img_line, clip((int)mapx[y1][x1], 0, img_line.width - 1),
                clip((int)mapy[y1][x1], 0, img_line.height - 1)) = 255;
#endif // CAR_DEBUG

            if (AT_IMAGE(&img_raw, x1, y1 - 1) < thres) break;
        }
        if (AT_IMAGE(&img_raw, x1, y1) >= thres) {
            findline_lefthand_adaptive(&img_raw, block_size, clip_value, x1, y1, circle_ipts, &circle_ipts_num);
        }
        else circle_ipts_num = 0;

        // 去畸变+透视变换（mapx，mapy，畸变坐标映射数组）
        for (int i = 0; i < circle_ipts_num; i++) {
            circle_rpts[i][0] = mapx[circle_ipts[i][1]][circle_ipts[i][0]];
            circle_rpts[i][1] = mapy[circle_ipts[i][1]][circle_ipts[i][0]];
        }
        circle_rpts_num = circle_ipts_num;

        // 边线滤波
        blur_points(circle_rpts, circle_rpts_num, circle_rptsb, (int)round(line_blur_kernel));
        circle_rptsb_num = circle_rpts_num;

        // 边线等距采样
        circle_rptss_num = sizeof(circle_rptss) / sizeof(circle_rptss[0]);
        resample_points(circle_rptsb, circle_rptsb_num, circle_rptss, &circle_rptss_num, 2 * sample_dist * pixel_per_meter);

        // 边线局部角度变化率
        local_angle_points(circle_rptss, circle_rptss_num, circle_rptsa, (int)round(angle_dist / sample_dist));
        circle_rptsa_num = circle_rptss_num;

        // 角度变化率非极大抑制
        nms_angle(circle_rptsa, circle_rptsa_num, circle_rptsan, (int)round(angle_dist / sample_dist) * 2 + 1);
        circle_rptsan_num = circle_rptsa_num;
    }
}

void check_circle() {
    if (circle_type == CIRCLE_NONE && Lpt0_found && !Lpt1_found && is_straight1) {
        track_type = TRACK_RIGHT;
    }

    if (circle_type == CIRCLE_NONE && !Lpt0_found && Lpt1_found && is_straight0) {
        track_type = TRACK_RIGHT;
    }

    // 非圆环模式下，单边L角点, 单边长直道
    // 不是圆环  拐点存在一个  一边长直道
    if (circle_type == CIRCLE_NONE && Lpt0_found && !Lpt1_found && is_straight1 && Lpt0_rpts0s_id < 0.6 / sample_dist) {
        circle_type = CIRCLE_LEFT_BEGIN;
    }
    if (circle_type == CIRCLE_NONE && !Lpt0_found && Lpt1_found && is_straight0 && Lpt1_rpts1s_id < 0.6 / sample_dist) {
        circle_type = CIRCLE_RIGHT_BEGIN;
    }
}

void run_circle() {
    //****  2023.11.6
    // 左环开始，寻外直道右线
    //**** track_type---巡线方式标志位
    if (circle_type == CIRCLE_LEFT_BEGIN) {
        if (circle_count++ > CIRCLE_COUNT) {
            circle_count = 0;
            circle_type = CIRCLE_NONE;
        }
        track_type = TRACK_RIGHT;

        //先丢左线后有线
        // 0.2 / 0.02 10个点 10cm
        if (rpts0s_num < 0.2 / sample_dist) { none_left_line++; }//***右环经历一个左线丢失的过程--右侧边线点的数量
        // 0.7 / 0.02 35个点 35cm
        if (/*rpts0s_num > 0.7 / sample_dist &&*/ none_left_line > 2 && !is_straight0 && is_straight1) {//***前面符合丢线并且线重新出现
            have_left_line++;
            if (have_left_line >= 1) {
                circle_count = 0;
                circle_type = CIRCLE_LEFT_IN;//***进环
                none_left_line = 0;
                have_left_line = 0;
            }
        }
    }
    //入环，寻内圆左线
    else if (circle_type == CIRCLE_LEFT_IN) {
        track_type = TRACK_RIGHT;
        circle_left_in_check();
        find_upcorners();
        if (circle_Lpt_found) {
            circle_rptss_num = circle_Lpt_rptss_id;
            if (circle_rptss[circle_Lpt_rptss_id][1] > LEFT_IN_UPLPY) {
                circle_ipts_num = circle_rpts_num = circle_rptsb_num = circle_rptss_num = circle_Lpt_rptss_id = 0;
                circle_type = CIRCLE_LEFT_RUNNING;
            }
            //draw_x(&img_line, circle_rptss[circle_Lpt_rptss_id][0], circle_rptss[circle_Lpt_rptss_id][1], 5, 255);
        }
        else if (circle_count++ > CIRCLE_COUNT) {
                circle_count = 0;
                circle_type = CIRCLE_NONE;
        }

    }
    //正常巡线，寻外圆右线
    else if (circle_type == CIRCLE_LEFT_RUNNING) {
        track_type = TRACK_LEFT;//TRACK_RIGHT;
        if (Lpt1_found) rpts1s_num = rptsc1_num = Lpt1_rpts1s_id;
        if (rpts0s_num <= 10 && rpts1s_num >= 60 && rpts1s_num <= 100  && !is_straight1) {
            track_type = TRACK_RIGHT;
        }
        else track_type = TRACK_LEFT;
        //外环拐点(右L点)
        // 0.65 / 0.02 32个点 32cm
        
        if (Lpt1_found && Lpt1_rpts1s_id < 0.65 / sample_dist) {
            circle_type = CIRCLE_LEFT_OUT;
        }
    }
    //出环，寻内圆
    else if (circle_type == CIRCLE_LEFT_OUT) {
        track_type = TRACK_LEFT;
        if(rpts0s_num < 3) {
            OUT_NONE_LINE = true;
        }else{
            OUT_NONE_LINE = false;
        }

        

        // double stdevRight = 0; // 边缘方差
        if (rpts1s_num < ROWSIMAGE / 4) {
            stdevRight = 1000;
        }
        vector<int> v_slope;
        int step = 10; // size/10;
        if (rpts1s_num > 60) {
            for (int i = step; i < 60; i += step) {
                if (rpts1s[i][0] - rpts1s[i - step][0]) {
                    v_slope.push_back(
                        (rpts1s[i][0] - rpts1s[i - step][0]) * 100 /
                        (rpts1s[i][1] - rpts1s[i - step][1])
                    );
                }

            }
        }
        else {
            for (int i = step; i < rpts1s_num; i += step) {
                if (rpts1s[i][0] - rpts1s[i - step][0]) {
                    v_slope.push_back(
                        (rpts1s[i][0] - rpts1s[i - step][0]) * 100 /
                        (rpts1s[i][1] - rpts1s[i - step][1])
                    );
                }

            }
        }

        if (v_slope.size() > 1) {
            double sum = accumulate(begin(v_slope), end(v_slope), 0.0);
            double mean = sum / v_slope.size(); // 均值
            double accum = 0.0;
            for_each(begin(v_slope), end(v_slope),
                [&](const double d) { accum += (d - mean) * (d - mean); });

            stdevRight = sqrt(accum / (v_slope.size() - 1)); // 方差
        }
        //COUT1(stdevRight);

        //右线为长直道
        if (is_straight1 || stdevRight < 5) {
            track_type = TRACK_RIGHT;
            circle_type = CIRCLE_NONE;
        }
    }
    //走过圆环，寻右线
    else if (circle_type == CIRCLE_LEFT_END) {
        track_type = TRACK_RIGHT;
        //左线先丢后有
        // 0.2 / 0.02 10个点 10cm
        if (rpts0s_num < 0.2 / sample_dist) { none_left_line++; }
        // 0.7 / 0.02 35个点 35cm
        if (rpts0s_num > 0.7 / sample_dist && none_left_line > 3) {
            circle_type = CIRCLE_NONE;
            none_left_line = 0;
        }
    }
    //右环控制，前期寻左直道
    else if (circle_type == CIRCLE_RIGHT_BEGIN) {
        if (circle_count++ > CIRCLE_COUNT) {
            circle_count = 0;
            circle_type = CIRCLE_NONE;
        }
        track_type = TRACK_LEFT;

        //先丢右线后有线
        // 0.2 / 0.02 10个点 10cm
        if (rpts1s_num < 0.2 / sample_dist) { none_right_line++; }
        // 0.7 / 0.02 50个点 50cm
        if (/*rpts1s_num > 0.7 / sample_dist && */none_right_line > 2 && is_straight0 && !is_straight1) {
            have_right_line++;
            if (have_right_line >= 1) {
                circle_count = 0;
                circle_type = CIRCLE_RIGHT_IN;
                none_right_line = 0;
                have_right_line = 0;
            }
        }
    }
    //入右环，寻右内圆环
    else if (circle_type == CIRCLE_RIGHT_IN) {
        track_type = TRACK_LEFT;
        circle_right_in_check();
        find_upcorners();
        if (circle_Lpt_found) {
            circle_rptss_num = circle_Lpt_rptss_id;
            if (circle_rptss[circle_Lpt_rptss_id][1] > RIGHT_IN_UPLPY) {
                circle_ipts_num = circle_rpts_num = circle_rptsb_num = circle_rptss_num = circle_Lpt_rptss_id = 0;
                circle_type = CIRCLE_RIGHT_RUNNING;
            }
            //draw_x(&img_line, circle_rptss[circle_Lpt_rptss_id][0], circle_rptss[circle_Lpt_rptss_id][1], 5, 255);
        }
        else if (circle_count++ > CIRCLE_COUNT) {
            circle_count = 0;
            circle_type = CIRCLE_NONE;
        }
    }
    //正常巡线，寻外圆左线
    else if (circle_type == CIRCLE_RIGHT_RUNNING) {
        track_type = TRACK_RIGHT;//TRACK_LEFT;
        //外环存在拐点,可再加拐点距离判据(左L点)
        if (Lpt0_found) rpts0s_num = rptsc0_num = Lpt0_rpts0s_id;
        if (rpts1s_num <= 10 && rpts0s_num >= 60 && rpts0s_num <= 100 && !is_straight0) {
            track_type = TRACK_LEFT;
        }
        else track_type = TRACK_RIGHT;
        // 0.65 / 0.02 32个点 32cm
        if (Lpt0_found && Lpt0_rpts0s_id < 0.65 / sample_dist) {
            circle_type = CIRCLE_RIGHT_OUT;
        }
    }
    //出环，寻内圆
    else if (circle_type == CIRCLE_RIGHT_OUT) {
        track_type = TRACK_RIGHT;
        if(rpts1s_num < 3) {
            OUT_NONE_LINE = true;
        }else{
            OUT_NONE_LINE = false;
        }


        // double stdevLeft = 0; // 边缘方差
        if (rpts0s_num < ROWSIMAGE / 4) {
            stdevLeft = 1000;
        }
        vector<int> v_slope;
        int step = 10; // size/10;
        if (rpts0s_num > 60) {
            for (int i = step; i < 60; i += step) {
                if (rpts0s[i][0] - rpts0s[i - step][0]) {
                    v_slope.push_back(
                        (rpts0s[i][0] - rpts0s[i - step][0]) * 100 /
                        (rpts0s[i][1] - rpts0s[i - step][1])
                    );
                }

            }
        }
        else {
            for (int i = step; i < rpts0s_num; i += step) {
                if (rpts0s[i][0] - rpts0s[i - step][0]) {
                    v_slope.push_back(
                        (rpts0s[i][0] - rpts0s[i - step][0]) * 100 /
                        (rpts0s[i][1] - rpts0s[i - step][1])
                    );
                }
            }
        }

        if (v_slope.size() > 1) {
            double sum = accumulate(begin(v_slope), end(v_slope), 0.0);
            double mean = sum / v_slope.size(); // 均值
            double accum = 0.0;
            for_each(begin(v_slope), end(v_slope),
                [&](const double d) { accum += (d - mean) * (d - mean); });

            stdevLeft = sqrt(accum / (v_slope.size() - 1)); // 方差
        }
        //        COUT1(stdevLeft);
                //左长度加倾斜角度  应修正左右线找到且为直线
                //if((rpts1s_num >100 && !Lpt1_found))  {have_right_line++;}
        if (is_straight0 || stdevLeft < 5) {
            track_type = TRACK_LEFT;
            circle_type = CIRCLE_NONE;
            //            circle_type = CIRCLE_RIGHT_END;
        }
    }
    //走过圆环，寻左线
    else if (circle_type == CIRCLE_RIGHT_END) {
        track_type = TRACK_LEFT;
        //左线先丢后有
        // 0.2 / 0.02 10个点 10cm
        if (rpts1s_num < 0.2 / sample_dist) { none_right_line++; }
        // 1 / 0.02 50个点 50cm
        if (rpts1s_num > 1.0 / sample_dist && none_right_line > 2) {
            circle_type = CIRCLE_NONE;
            none_right_line = 0;
        }
    }
}

// 绘制圆环模式下的调试图像
void draw_circle() {

}

