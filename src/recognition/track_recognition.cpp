#include "track_recognition.h"
#include "../src/main.h"
#include "../code/headfile.h"
#include "../code/camera_param.h"
#include "../code/imgproc.h"

#include <vector>
#include <numeric>
using namespace std;

// 原图左右边线
int ipts0[POINTS_MAX_LEN][2];
int ipts1[POINTS_MAX_LEN][2];
int ipts0_num, ipts1_num;
// 变换后左右边线
float rpts0[POINTS_MAX_LEN][2];
float rpts1[POINTS_MAX_LEN][2];
int rpts0_num, rpts1_num;
// 变换后左右边线+滤波
float rpts0b[POINTS_MAX_LEN][2];
float rpts1b[POINTS_MAX_LEN][2];
int rpts0b_num, rpts1b_num;
// 变换后左右边线+等距采样
float rpts0s[POINTS_MAX_LEN][2];
float rpts1s[POINTS_MAX_LEN][2];
int rpts0s_num, rpts1s_num;
// 左右边线局部角度变化率
float rpts0a[POINTS_MAX_LEN];
float rpts1a[POINTS_MAX_LEN];
int rpts0a_num, rpts1a_num;
// 左右边线局部角度变化率+非极大抑制
float rpts0an[POINTS_MAX_LEN];
float rpts1an[POINTS_MAX_LEN];
int rpts0an_num, rpts1an_num;
// 左/右中线
float rptsc0[POINTS_MAX_LEN][2];
float rptsc1[POINTS_MAX_LEN][2];
int rptsc0_num, rptsc1_num;

// 中线
float(*rpts)[2];
int rpts_num;
// 归一化中线
float rptsn[POINTS_MAX_LEN][2];
int rptsn_num;

// L角点-远处
int Lpt0_far_rpts0s_id, Lpt1_far_rpts1s_id;
bool Lpt0_far_found, Lpt1_far_found;

// L角点
int Lpt0_rpts0s_id, Lpt1_rpts1s_id;
bool Lpt0_found, Lpt1_found;

// 长直道
bool is_straight0 = false;
bool is_straight1 = false;

//// 弯道
//bool is_turn0, is_turn1;

double stdevLeft = 0; // 边缘方差
double stdevRight = 0; // 边缘方差

double stdevLeft_far = 0; // 边缘方差
double stdevRight_far = 0; // 边缘方差

double Lradius = 1000;
double Rradius = 1000;
double Lradius_far = 1000;
double Rradius_far = 1000;
double Lradius_ffar = 1000;
double Rradius_ffar = 1000;

const int STDEVLNUM = 80;
const int STDEVRNUM = 80;

const double straightL_angle = 10.0;
const double straightR_angle = 10.0;

const double straightL_stdev = 10.0;
const double straightR_stdev = 10.0;

// 当前巡线模式
enum track_type_e track_type = TRACK_RIGHT;

using namespace cv;
uint8_t img_data[ROWSIMAGE][COLSIMAGE];
image_t img_raw((uint8_t*)img_data, COLSIMAGE, ROWSIMAGE);
void frameTOimg_raw(const Mat& frame) {
    for (int c = 0; c < COLSIMAGE; c++) {
        for (int r = 0; r < ROWSIMAGE; r++) {
            AT_IMAGE(&img_raw, (int)c, (int)r) = (int)frame.at<uchar>(r, c);
        }
    }
}


void get_normal_line() {
    // 原图找左右边线
    int x1 = COLSIMAGE / 2 - begin_x, y1 = begin_y;
    // ipts0_num = sizeof(ipts0) / sizeof(ipts0[0]);
    ipts0_num = 240;

    // x1 < 115根据填充白色多边形循迹边界判断
    for (; x1 > 1; x1--) if (AT_IMAGE(&img_raw, x1 - 1, y1) < thres && x1 < X1_BOUNDARY) break;
    if (AT_IMAGE(&img_raw, x1, y1) >= thres) {
        findline_lefthand_adaptive(&img_raw, block_size, clip_value, x1, y1, ipts0, &ipts0_num);
    }
    else ipts0_num = 0;

    int x2 = COLSIMAGE / 2 + begin_x, y2 = begin_y;
    // ipts1_num = sizeof(ipts1) / sizeof(ipts1[0]);
    ipts1_num = 240;

    // x2 > 213根据填充白色多边形循迹边界判断
    for (; x2 < COLSIMAGE - 2; x2++) if (AT_IMAGE(&img_raw, x2 + 1, y2) < thres && x2 > X2_BOUNDARY) break;
    if (AT_IMAGE(&img_raw, x2, y2) >= thres) {
        findline_righthand_adaptive(&img_raw, block_size, clip_value, x2, y2, ipts1, &ipts1_num);
    }
    else ipts1_num = 0;
}

void line_process() {
    // 去畸变+透视变换（mapx，mapy，畸变坐标映射数组）
    for (int i = 0; i < ipts0_num; i++) {
        rpts0[i][0] = mapx[ipts0[i][1]][ipts0[i][0]];
        rpts0[i][1] = mapy[ipts0[i][1]][ipts0[i][0]];
    }
    rpts0_num = ipts0_num;
    for (int i = 0; i < ipts1_num; i++) {
        rpts1[i][0] = mapx[ipts1[i][1]][ipts1[i][0]];
        rpts1[i][1] = mapy[ipts1[i][1]][ipts1[i][0]];
    }
    rpts1_num = ipts1_num;

    // 边线滤波
    blur_points(rpts0, rpts0_num, rpts0b, (int)round(line_blur_kernel));
    rpts0b_num = rpts0_num;
    blur_points(rpts1, rpts1_num, rpts1b, (int)round(line_blur_kernel));
    rpts1b_num = rpts1_num;

    // 边线等距采样
    // rpts0s_num = sizeof(rpts0s) / sizeof(rpts0s[0]);
    rpts0s_num = 240;
    resample_points(rpts0b, rpts0b_num, rpts0s, &rpts0s_num, 2 * sample_dist * pixel_per_meter);
    // rpts1s_num = sizeof(rpts1s) / sizeof(rpts1s[0]);
    rpts1s_num = 240;
    resample_points(rpts1b, rpts1b_num, rpts1s, &rpts1s_num, 2 * sample_dist * pixel_per_meter);
    
    // 边线局部角度变化率
    local_angle_points(rpts0s, rpts0s_num, rpts0a, (int)round(angle_dist / sample_dist));
    rpts0a_num = rpts0s_num;
    local_angle_points(rpts1s, rpts1s_num, rpts1a, (int)round(angle_dist / sample_dist));
    rpts1a_num = rpts1s_num;

    // 角度变化率非极大抑制
    nms_angle(rpts0a, rpts0a_num, rpts0an, (int)round(angle_dist / sample_dist) * 2 + 1);
    rpts0an_num = rpts0a_num;
    nms_angle(rpts1a, rpts1a_num, rpts1an, (int)round(angle_dist / sample_dist) * 2 + 1);
    rpts1an_num = rpts1a_num;

    find_corners();

    // 左右中线跟踪
    track_leftline(rpts0s, rpts0s_num, rptsc0, (int)round(angle_dist / sample_dist), pixel_per_meter * ROAD_WIDTH / 2);
    rptsc0_num = rpts0s_num;
    track_rightline(rpts1s, rpts1s_num, rptsc1, (int)round(angle_dist / sample_dist), pixel_per_meter * ROAD_WIDTH / 2);
    rptsc1_num = rpts1s_num;
    
    extern int LR_num1;
    extern int LR_num2;
    extern int LR_num3;
    
    if (rpts0s_num > LR_num3) {
        int Lid1 = MIN(LR_num1, rpts0s_num - 1);
        int Lid2 = MIN(LR_num2, rpts0s_num - 1);
        int Lid3 = MIN(LR_num3, rpts0s_num - 1);
        Lradius = radius_3pts(rpts0s[Lid1], rpts0s[Lid2], rpts0s[Lid3]);
    }else Lradius = 0;

    extern int RR_num1;
    extern int RR_num2;
    extern int RR_num3;

    if (rpts1s_num > RR_num3) {
        int Rid1 = MIN(RR_num1, rpts1s_num - 1);
        int Rid2 = MIN(RR_num2, rpts1s_num - 1);
        int Rid3 = MIN(RR_num3, rpts1s_num - 1);
        Rradius = radius_3pts(rpts1s[Rid1], rpts1s[Rid2], rpts1s[Rid3]);
    }else Rradius = 0;
    
    extern int LR_F_num1;
    extern int LR_F_num2;
    extern int LR_F_num3;

    int Lid1_far = MIN(LR_F_num1, rpts0s_num - 1);
    int Lid2_far = MIN(LR_F_num2, rpts0s_num - 1);
    int Lid3_far = MIN(LR_F_num3, rpts0s_num - 1);
    Lradius_far = radius_3pts(rpts0s[Lid1_far], rpts0s[Lid2_far], rpts0s[Lid3_far]);
   
    extern int RR_F_num1;
    extern int RR_F_num2;
    extern int RR_F_num3;

    int Rid1_far = MIN(RR_F_num1, rpts1s_num - 1);
    int Rid2_far = MIN(RR_F_num2, rpts1s_num - 1);
    int Rid3_far = MIN(RR_F_num3, rpts1s_num - 1);
    Rradius_far = radius_3pts(rpts1s[Rid1_far], rpts1s[Rid2_far], rpts1s[Rid3_far]);

    if(Lradius > 1e5)Lradius=1e5;
    if(Rradius > 1e5)Rradius=1e5;
    if(Lradius_far > 1e5)Lradius_far=1e5;
    if(Rradius_far > 1e5)Rradius_far=1e5;

    vector<int> v_slope;
    int step = 10; // size/10;

    if (rpts1s_num < ROWSIMAGE / 4) {
        stdevRight = 1000;
    }
    if (rpts1s_num > STDEVRNUM) {
        for (int i = step; i < STDEVRNUM; i += step) {
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
        stdevRight = 1000;
    }

    if (rpts0s_num < ROWSIMAGE / 4) {
        stdevLeft = 1000;
    }

    v_slope.clear();
    if (rpts0s_num > STDEVLNUM) {
        for (int i = step; i < STDEVLNUM; i += step) {
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
        stdevLeft = 1000;
    }

    if (stdevLeft <= 5 && rpts0s_num > 40) {
        is_straight0 = true;
    }
    else {
        is_straight0 = false;
    }

    if (stdevRight <= 5 && rpts1s_num > 40) {
        is_straight1 = true;
    }
    else {
        is_straight1 = false;
    }
}


void find_corners() {
    // 识别Y,L拐点
    Lpt0_far_found = Lpt1_far_found = Lpt0_found = Lpt1_found = false;
    // is_straight0 = rpts0s_num > 1.6 / sample_dist;//1为米数相当于  等距采样后的点大于这点数量
    // is_straight1 = rpts1s_num > 1.6 / sample_dist;
    for (int i = 0; i < rpts0s_num; i++) {//左边线的处理
        if (rpts0an[i] == 0) continue;//非极大值抑制
        int im1 = clip(i - (int)round(angle_dist / sample_dist), 0, rpts0s_num - 1);
        int ip1 = clip(i + (int)round(angle_dist / sample_dist), 0, rpts0s_num - 1);
        float conf = fabs(rpts0a[i]) - (fabs(rpts0a[im1]) + fabs(rpts0a[ip1])) / 2;//计算真实的角度
        // 1.2/ 0.02 60个点 60cm
        // 0.6/ 0.02 30个点 30cm
        //L角点(远)阈值
        if (Lpt0_far_found == false && 70. / 180. * PI < conf && conf < 120. / 180. * PI && i < aim_distance / sample_dist) {
            Lpt0_far_rpts0s_id = i;
            Lpt0_far_found = true;
        }
        //L角点阈值
        if (Lpt0_found == false && 70. / 180. * PI < conf && conf < 120. / 180. * PI && i < 0.8 / sample_dist) {//i < 0.8 / sample_dist 相当于让点在近处
            Lpt0_rpts0s_id = i;
            Lpt0_found = true;
        }
        //长直道阈值
        // if (conf > straightL_angle / 180. * PI && i < 1.6 / sample_dist && stdevLeft > straightL_stdev) is_straight0 = false;//有角度大于5度的角度时候不是长直道
        if (Lpt0_far_found == true && Lpt0_found == true && is_straight0 == false) break;
    }
    for (int i = 0; i < rpts1s_num; i++) {
        if (rpts1an[i] == 0) continue;
        int im1 = clip(i - (int)round(angle_dist / sample_dist), 0, rpts1s_num - 1);
        int ip1 = clip(i + (int)round(angle_dist / sample_dist), 0, rpts1s_num - 1);
        float conf = fabs(rpts1a[i]) - (fabs(rpts1a[im1]) + fabs(rpts1a[ip1])) / 2;
        // 1.2/ 0.02 60个点 60cm
        // 0.6/ 0.02 30个点 30cm
        if (Lpt1_far_found == false && 70. / 180. * PI < conf && conf < 120. / 180. * PI && i < aim_distance / sample_dist) {
            Lpt1_far_rpts1s_id = i;
            Lpt1_far_found = true;
        }
        if (Lpt1_found == false && 70. / 180. * PI < conf && conf < 120. / 180. * PI && i < 0.8 / sample_dist) {
            Lpt1_rpts1s_id = i;
            Lpt1_found = true;
        }

        // if (conf > straightR_angle / 180. * PI && i < 1.6 / sample_dist && stdevRight > straightR_stdev) is_straight1 = false;

        if (Lpt1_far_found == true && Lpt1_found == true && is_straight1 == false) break;
    }
    // L点-远 二次检查,依据两角点距离及角点后张开特性
    if (Lpt0_far_found && Lpt1_far_found) {
        float dx = rpts0s[Lpt0_far_rpts0s_id][0] - rpts1s[Lpt1_far_rpts1s_id][0];
        float dy = rpts0s[Lpt0_far_rpts0s_id][1] - rpts1s[Lpt1_far_rpts1s_id][1];
        float dn = sqrtf(dx * dx + dy * dy);
        if (fabs(dn - 0.45 * pixel_per_meter) < 0.15 * pixel_per_meter) {
            float dwx = rpts0s[clip(Lpt0_far_rpts0s_id + 50, 0, rpts0s_num - 1)][0] -
                rpts1s[clip(Lpt1_far_rpts1s_id + 50, 0, rpts1s_num - 1)][0];
            float dwy = rpts0s[clip(Lpt0_far_rpts0s_id + 50, 0, rpts0s_num - 1)][1] -
                rpts1s[clip(Lpt1_far_rpts1s_id + 50, 0, rpts1s_num - 1)][1];
            float dwn = sqrtf(dwx * dwx + dwy * dwy);
            if (!(dwn > 0.7 * pixel_per_meter &&
                rpts0s[clip(Lpt0_far_rpts0s_id + 50, 0, rpts0s_num - 1)][0] < rpts0s[Lpt0_far_rpts0s_id][0] &&
                rpts1s[clip(Lpt1_far_rpts1s_id + 50, 0, rpts1s_num - 1)][0] > rpts1s[Lpt1_far_rpts1s_id][0])) {
                Lpt0_far_found = Lpt1_far_found = false;
            }
        }
        else {
            Lpt0_far_found = Lpt1_far_found = false;
        }
    }

    // L点二次检查，车库模式不检查, 依据L角点距离及角点后张开特性
    if (true/*garage_type == GARAGE_NONE*/) {
        if (Lpt0_found && Lpt1_found) {
            float dx = rpts0s[Lpt0_rpts0s_id][0] - rpts1s[Lpt1_rpts1s_id][0];
            float dy = rpts0s[Lpt0_rpts0s_id][1] - rpts1s[Lpt1_rpts1s_id][1];
            float dn = sqrtf(dx * dx + dy * dy);
            if (fabs(dn - 0.45 * pixel_per_meter) < 0.15 * pixel_per_meter) {
                float dwx = rpts0s[clip(Lpt0_rpts0s_id + 50, 0, rpts0s_num - 1)][0] -
                    rpts1s[clip(Lpt1_rpts1s_id + 50, 0, rpts1s_num - 1)][0];
                float dwy = rpts0s[clip(Lpt0_rpts0s_id + 50, 0, rpts0s_num - 1)][1] -
                    rpts1s[clip(Lpt1_rpts1s_id + 50, 0, rpts1s_num - 1)][1];
                float dwn = sqrtf(dwx * dwx + dwy * dwy);
                if (!(dwn > 0.7 * pixel_per_meter &&
                    rpts0s[clip(Lpt0_rpts0s_id + 50, 0, rpts0s_num - 1)][0] < rpts0s[Lpt0_rpts0s_id][0] &&
                    rpts1s[clip(Lpt1_rpts1s_id + 50, 0, rpts1s_num - 1)][0] > rpts1s[Lpt1_rpts1s_id][0])) {
                    Lpt0_found = Lpt1_found = false;
                }
            }
            else {
                Lpt0_found = Lpt1_found = false;
            }
        }
    }
    
    if (Lpt0_far_found) rpts0s_num = clip(Lpt0_far_rpts0s_id, 0, rpts0s_num - 1);
    if (Lpt1_far_found) rpts1s_num = clip(Lpt1_far_rpts1s_id, 0, rpts1s_num - 1);
    if (Lpt0_found && Lpt0_rpts0s_id > 30) rpts0s_num = clip(Lpt0_rpts0s_id, 0, rpts0s_num - 1);
    if (Lpt1_found && Lpt0_rpts0s_id > 30) rpts1s_num = clip(Lpt1_rpts1s_id, 0, rpts1s_num - 1);
}

void track_switch() {
    // 单侧线少，切换巡线方向  切外向圆
    if (rpts0s_num < rpts1s_num / 2 && rpts0s_num < 60) {
        track_type = TRACK_RIGHT;
    }
    else if (rpts1s_num < rpts0s_num / 2 && rpts1s_num < 60) {
        track_type = TRACK_LEFT;
    }
    else if (rpts0s_num < 20 && rpts1s_num > rpts0s_num) {
        track_type = TRACK_RIGHT;
    }
    else if (rpts1s_num < 20 && rpts0s_num > rpts1s_num) {
        track_type = TRACK_LEFT;
    }

}

void show_line() {
    //////////////////画线 2023年11月21日
    // extern cv::Mat lineFrame;
    //for (int c = 0; c < COLSIMAGE; c++) {
    //    for (int r = 0; r < ROWSIMAGE; r++) {
    //        if (AT_IMAGE(&img_raw, c, r) < 140) {
    //            circle(lineFrame, Point(c, r), 1, Scalar(0, 0, 0));
    //        }
    //        else {
    //            circle(lineFrame, Point(c, r), 1, Scalar(255, 255, 255));
    //        }
    //    }
    //}
    // for (int i = 0; i < ipts0_num; i++) {
    //     int x = ipts0[i][0];
    //     int y = ipts0[i][1];
    //     int current_value = AT_IMAGE(&img_raw, x, y);//当前灰度值x,y

    //     if (current_value < 140) {
    //         cv::circle(lineFrame, cv::Point(x, y), 1, cv::Scalar(0, 0, 0));
    //     }
    //     else {
    //         cv::circle(lineFrame, cv::Point(x, y), 1, cv::Scalar(255, 255, 255));
    //     }
    // }
    // for (int i = 0; i < ipts1_num; i++) {
    //     int x = ipts1[i][0];
    //     int y = ipts1[i][1];
    //     int current_value = AT_IMAGE(&img_raw, x, y);//当前灰度值x,y

    //     if (current_value < 140) {
    //         cv::circle(lineFrame, cv::Point(x, y), 1, cv::Scalar(0, 0, 0));
    //     }
    //     else {
    //         cv::circle(lineFrame, cv::Point(x, y), 1, cv::Scalar(255, 255, 255));
    //     }
    // }
    // imshow("lineFrame", lineFrame);
    
    // 绘制逆透视后的道路线
    extern cv::Mat nitoushi;
    for (int c = 0; c < COLSIMAGE; c++) {
        for (int r = 0; r < ROWSIMAGE; r++) {
            if (AT_IMAGE(&img_line, c, r) < 140) {
                circle(nitoushi, Point(c, r), 1, Scalar(0, 0, 0));
            }
            else {
                circle(nitoushi, Point(c, r), 1, Scalar(255, 255, 255));
            }
        }
    }
#ifdef CAR_SAVEIMG
    static int i = 1;
    std::string filename = "../frame/nitoushi" + std::to_string(i++) + ".jpg";
    imwrite(filename, nitoushi);
#endif
#ifdef CAR_SHOW
    imshow("nitoushi", nitoushi);
#endif
}
