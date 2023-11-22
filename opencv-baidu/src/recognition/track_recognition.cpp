#include "track_recognition.h"

float thres = 140;      //二值化阈值
float block_size = 7;   //
float clip_value = 2;   //

//float begin_x = 32;     //寻线起始x坐标（COLSIMAGE / 2 - begin_x）
//float begin_y = 167;    //寻线起始y坐标

float begin_x = 15;     //寻线起始x坐标（COLSIMAGE / 2 - begin_x）
float begin_y = 167;    //寻线起始y坐标


float line_blur_kernel = 7;
float pixel_per_meter = 102;//平移像素，拟合中线
float sample_dist = 0.02;
float angle_dist = 0.2;
float far_rate = 0.5;
float aim_distance = 0.68;
bool adc_cross = false;

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


using namespace cv;
uint8_t img_data[ROWSIMAGE][COLSIMAGE];
image_t img_raw((uint8_t*)img_data, COLSIMAGE, ROWSIMAGE);
void frameTOimg_raw(const Mat& frame) {
    for (int c = 0; c < COLSIMAGE; c++) {
        for (int r = 0; r < ROWSIMAGE; r++) {
            AT_IMAGE(&img_raw, (int)c, (int)r) = (int)frame.at<uchar>(r, c);
            /*if ((int)frame.at<uchar>(r, c) < 5) {
                AT_IMAGE(&img_raw, (int)r, (int)c) = 0;
            }
            else {
                AT_IMAGE(&img_raw, (int)r, (int)c) = 255;
            }*/
        }
    }
    //cv::Mat ff = cv::Mat::zeros(cv::Size(320, 240), CV_8UC1);

    //for (int c = 0; c < COLSIMAGE; c++) {
    //    for (int r = 0; r < ROWSIMAGE; r++) {
    //        if (AT_IMAGE(&img_raw, (int)c, (int)r) > 140) {
    //            cv::circle(ff, cv::Point(c, r), 1, cv::Scalar(255, 255, 255));
    //        }
    //        else {
    //            cv::circle(ff, cv::Point(c, r), 1, cv::Scalar(0, 0, 0));
    //        }
    //    }
    //}
    //imshow("f", ff);
    //waitKey(1);
}
void process_image() {
    // 原图找左右边线
    int x1 = COLSIMAGE / 2 - begin_x, y1 = begin_y;
    ipts0_num = sizeof(ipts0) / sizeof(ipts0[0]);
    for (; x1 > 0; x1--) if (AT_IMAGE(&img_raw, x1 - 1, y1) < thres) break;
    if (AT_IMAGE(&img_raw, x1, y1) >= thres) {
        findline_lefthand_adaptive(&img_raw, block_size, clip_value, x1, y1, ipts0, &ipts0_num);
    }
    else ipts0_num = 0;

    int x2 = COLSIMAGE / 2 + begin_x, y2 = begin_y;
    ipts1_num = sizeof(ipts1) / sizeof(ipts1[0]);
    for (; x2 < COLSIMAGE - 1; x2++) if (AT_IMAGE(&img_raw, x2 + 1, y2) < thres) break;
    if (AT_IMAGE(&img_raw, x2, y2) >= thres) {
        findline_righthand_adaptive(&img_raw, block_size, clip_value, x2, y2, ipts1, &ipts1_num);
    }
    else ipts1_num = 0;
    
    //////////////////画线 2023年11月21日
    cv::Mat lineFrame = cv::Mat::zeros(cv::Size(320, 240), CV_8UC1);
    for (int i = 0; i < ipts0_num; i++) {
        int x = ipts0[i][0];
        int y = ipts0[i][1];
        int current_value = AT_IMAGE(&img_raw, x, y);//当前灰度值x,y
    
        if (current_value < 140) {
            cv::circle(lineFrame, cv::Point(x, y), 1, cv::Scalar(0, 0, 0));
        }
        else {
            cv::circle(lineFrame, cv::Point(x, y), 1, cv::Scalar(255, 255, 255));
        }
    }
    for (int i = 0; i < ipts1_num; i++) {
        int x = ipts1[i][0];
        int y = ipts1[i][1];
        int current_value = AT_IMAGE(&img_raw, x, y);//当前灰度值x,y

        if (current_value < 140) {
            cv::circle(lineFrame, cv::Point(x, y), 1, cv::Scalar(0, 0, 0));
        }
        else {
            cv::circle(lineFrame, cv::Point(x, y), 1, cv::Scalar(255, 255, 255));
        }
    }
    imshow("lineFrame", lineFrame);
    //////////////////边线 2023年11月21日

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
    Mat nitoushi = Mat::zeros(Size(320, 240), CV_8UC1);
    for (int i = 0; i < rpts0_num; i++) {
        int x = rpts0[i][0];
        int y = rpts0[i][1];
        if (AT_IMAGE(&img_raw, ipts0[i][0], ipts0[i][1]) < 5) {
            circle(nitoushi, Point(x, y), 1, Scalar(0, 0, 0));
        }
        else {
            circle(nitoushi, Point(x, y), 1, Scalar(255, 255, 255));
        }
    }
    for (int i = 0; i < rpts1_num; i++) {
        int x = rpts1[i][0];
        int y = rpts1[i][1];
        if (AT_IMAGE(&img_raw, ipts1[i][0], ipts1[i][1]) < 5) {
            circle(nitoushi, Point(x, y), 1, Scalar(0, 0, 0));
        }
        else {
            circle(nitoushi, Point(x, y), 1, Scalar(255, 255, 255));
        }

    }
    imshow("nitoushi", nitoushi);

    // 边线滤波
    blur_points(rpts0, rpts0_num, rpts0b, (int)round(line_blur_kernel));
    rpts0b_num = rpts0_num;
    blur_points(rpts1, rpts1_num, rpts1b, (int)round(line_blur_kernel));
    rpts1b_num = rpts1_num;

    // 边线等距采样
    rpts0s_num = sizeof(rpts0s) / sizeof(rpts0s[0]);
    resample_points(rpts0b, rpts0b_num, rpts0s, &rpts0s_num, sample_dist * pixel_per_meter);
    rpts1s_num = sizeof(rpts1s) / sizeof(rpts1s[0]);
    resample_points(rpts1b, rpts1b_num, rpts1s, &rpts1s_num, sample_dist * pixel_per_meter);

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

    // 左右中线跟踪
    track_leftline(rpts0s, rpts0s_num, rptsc0, (int)round(angle_dist / sample_dist), pixel_per_meter * ROAD_WIDTH / 2);
    rptsc0_num = rpts0s_num;
    track_rightline(rpts1s, rpts1s_num, rptsc1, (int)round(angle_dist / sample_dist), pixel_per_meter * ROAD_WIDTH / 2);
    rptsc1_num = rpts1s_num;
}
