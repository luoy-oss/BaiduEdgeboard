#include "circle_recognition.h"
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
        "CIRCLE_LEFT_RUNNING", "CIRCLE_RIGHT_RUNNING",
        "CIRCLE_LEFT_IN", "CIRCLE_RIGHT_IN",
        "CIRCLE_LEFT_OUT", "CIRCLE_RIGHT_OUT",
        "CIRCLE_LEFT_END", "CIRCLE_RIGHT_END",
};

//// 编码器，用于防止一些重复触发等。
//int64_t circle_encoder;

int none_left_line = 0, none_right_line = 0;
int have_left_line = 0, have_right_line = 0;

void check_circle() {
    // 非圆环模式下，单边L角点, 单边长直道
    // 不是圆环  拐点存在一个  一边长直道
    if (circle_type == CIRCLE_NONE && Lpt0_found && !Lpt1_found && is_straight1) {
        circle_type = CIRCLE_LEFT_BEGIN;
    }
    if (circle_type == CIRCLE_NONE && !Lpt0_found && Lpt1_found && is_straight0) {
        circle_type = CIRCLE_RIGHT_BEGIN;
    }
}

void run_circle() {
    //****  2023.11.6
    // 左环开始，寻外直道右线
    //**** track_type---巡线方式标志位
    if (circle_type == CIRCLE_LEFT_BEGIN) {
        track_type = TRACK_RIGHT;

        //先丢左线后有线
        if (rpts0s_num < 0.2 / sample_dist) { none_left_line++; }//***右环经历一个左线丢失的过程--右侧边线点的数量
        if (rpts0s_num > 1.0 / sample_dist && none_left_line > 2) {//***前面符合丢线并且线重新出现
            have_left_line++;
            if (have_left_line > 1) {
                circle_type = CIRCLE_LEFT_IN;//***进环
                none_left_line = 0;
                have_left_line = 0;
            }
        }
    }
    //入环，寻内圆左线
    else if (circle_type == CIRCLE_LEFT_IN) {
        track_type = TRACK_LEFT;

        double stdevRight = 0; // 边缘方差
        if (rpts1s_num < ROWSIMAGE / 4) {
            stdevRight = 1000;
        }
        vector<int> v_slope;
        int step = 10; // size/10;
        if (rpts1s_num > 100) {
            for (int i = step; i < 100; i += step) {
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
        else {
            stdevRight = 0;
        }
        if (stdevRight == 0) {
            none_right_line++;
        }

        /*if (none_right_line > 3) {
            cout <<"stdevRight::" << stdevRight << endl;
        }*/

        //编码器打表过1/4圆   应修正为右线为转弯无拐点
        //COUT1(rpts0s_num);
        //左点数小于100
        if (rpts0s_num < 100/*0.2 / sample_dist*/ && none_right_line > 3 && stdevRight > 40/*||
            current_encoder - circle_encoder >= ENCODER_PER_METER * (3.14 * 1 / 2)*/) {
            circle_type = CIRCLE_LEFT_RUNNING;
            none_right_line = 0;
        }
        //***编码器打脚并且右线点数减少进入环内
    }
    //正常巡线，寻外圆右线
    else if (circle_type == CIRCLE_LEFT_RUNNING) {
        track_type = TRACK_RIGHT;

        if (Lpt1_found) rpts1s_num = rptsc1_num = Lpt1_rpts1s_id;
        //外环拐点(右L点)
        if (Lpt1_found && Lpt1_rpts1s_id < 0.4 / sample_dist) {
            circle_type = CIRCLE_LEFT_OUT;
        }
    }
    //出环，寻内圆
    else if (circle_type == CIRCLE_LEFT_OUT) {
        track_type = TRACK_LEFT;

        //右线为长直道
        if (is_straight1) {
            circle_type = CIRCLE_LEFT_END;
        }
    }
    //走过圆环，寻右线
    else if (circle_type == CIRCLE_LEFT_END) {
        track_type = TRACK_RIGHT;

        //左线先丢后有
        if (rpts0s_num < 0.2 / sample_dist) { none_left_line++; }
        if (rpts0s_num > 1.0 / sample_dist && none_left_line > 3) {
            circle_type = CIRCLE_NONE;
            none_left_line = 0;
        }
    }
    //右环控制，前期寻左直道
    else if (circle_type == CIRCLE_RIGHT_BEGIN) {
        track_type = TRACK_LEFT;

        //先丢右线后有线
        if (rpts1s_num < 0.2 / sample_dist) { none_right_line++; }
        if (rpts1s_num > 1.0 / sample_dist && none_right_line > 2) {
            have_right_line++;
            if (have_right_line > 1) {
                circle_type = CIRCLE_RIGHT_IN;
                none_right_line = 0;
                have_right_line = 0;
            }
        }
    }
    //入右环，寻右内圆环
    else if (circle_type == CIRCLE_RIGHT_IN) {
        track_type = TRACK_RIGHT;

        double stdevLeft = 0; // 边缘方差
        if (rpts0s_num < ROWSIMAGE / 4) {
            stdevLeft = 1000;
        }
        vector<int> v_slope;
        int step = 10; // size/10;
        if (rpts0s_num > 100) {
            for (int i = step; i < 100; i += step) {
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
        else {
            stdevLeft = 0;
        }
        if (stdevLeft == 0) {
            none_left_line++;
        }
        /*if (none_left_line > 3) {
            cout <<"stdevLeft::" << stdevLeft << endl;
        }*/
        //编码器打表过1/4圆   应修正为左线为转弯无拐点
        //右点数小于100
        if (rpts1s_num < 100/*0.2 / sample_dist*/ && none_left_line > 3 && stdevLeft > 40 /*||
            current_encoder - circle_encoder >= ENCODER_PER_METER * (3.14 * 1 / 2)*/) {
            circle_type = CIRCLE_RIGHT_RUNNING;
            none_left_line = 0;
        }

    }
    //正常巡线，寻外圆左线
    else if (circle_type == CIRCLE_RIGHT_RUNNING) {
        track_type = TRACK_LEFT;

        //外环存在拐点,可再加拐点距离判据(左L点)
        if (Lpt0_found) rpts0s_num = rptsc0_num = Lpt0_rpts0s_id;
        if (Lpt0_found && Lpt0_rpts0s_id < 0.4 / sample_dist) {
            circle_type = CIRCLE_RIGHT_OUT;
        }
    }
    //出环，寻内圆
    else if (circle_type == CIRCLE_RIGHT_OUT) {
        track_type = TRACK_RIGHT;

        //左长度加倾斜角度  应修正左右线找到且为直线
        //if((rpts1s_num >100 && !Lpt1_found))  {have_right_line++;}
        if (is_straight0) {
            circle_type = CIRCLE_RIGHT_END;
        }
    }
    //走过圆环，寻左线
    else if (circle_type == CIRCLE_RIGHT_END) {
        track_type = TRACK_LEFT;

        //左线先丢后有
        if (rpts1s_num < 0.2 / sample_dist) { none_right_line++; }
        if (rpts1s_num > 1.0 / sample_dist && none_right_line > 2) {
            circle_type = CIRCLE_NONE;
            none_right_line = 0;
        }
    }
}

// 绘制圆环模式下的调试图像
void draw_circle() {

}

