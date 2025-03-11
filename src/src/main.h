#ifndef __MAIN_H__
#define __MAIN_H__

// #define CAR_SHOW
// #define CAR_SAVEIMG
// #define CAR_DEBUG
//#define CIRCLE_DEBUG
//#define CROSS_DEBUG
//#define GARAGE_DEBUG

#include "../code/headfile.h"
#include "../code/imgproc.h"
#include "../code/utils.h"
#include "../code/common.h"

#define COUT1(x) (std::cout<<(x)<<std::endl)
#define COUT2(x,y) (std::cout<<(x)<<","<<(y)<<std::endl)
#define MAT_AT_SET(dst,r,c,new_b,new_g,new_r) dst.at<cv::Vec3b>(r, c)[0] = new_b;\
dst.at<cv::Vec3b>(r, c)[1] = new_g;\
dst.at<cv::Vec3b>(r, c)[2] = new_r

#define MAT_SET(dst,r,c,new_b,new_g,new_r) dst.

extern float thres;                 // 二值化阈值，主要用于找起始点(边线使用自适应阈值，不使用该阈值)
extern float block_size;            // 自适应阈值的block大小
extern float clip_value;            // 自适应阈值的阈值裁减量
extern float begin_x;               // 起始点距离图像中心的左右偏移量
extern float begin_y;               // 起始点距离图像底部的上下偏移量
extern const int X1_BOUNDARY;
extern const int X2_BOUNDARY;
extern float line_blur_kernel;      // 边线三角滤波核的大小
extern float pixel_per_meter;       // 俯视图中，每个像素对应的长度
extern float sample_dist;           // 边线等距采样的间距
extern float angle_dist;            // 计算边线转角时，三个计算点的距离
extern float aim_distance;          // 预锚点长度

extern image_t img_raw;
extern image_t img_thres;
extern image_t img_line;

extern bool line_show_sample;
extern bool line_show_blur;
extern bool track_left;

extern double angle;

#define ROAD_WIDTH      (0.45)
#define POINTS_MAX_LEN  (ROWSIMAGE)

#define FAR_POINTS_MAX_LEN  (POINTS_MAX_LEN)

// 原图左右边线
extern int ipts0[POINTS_MAX_LEN][2];
extern int ipts1[POINTS_MAX_LEN][2];
extern int ipts0_num, ipts1_num;
// 变换后左右边线
extern float rpts0[POINTS_MAX_LEN][2];
extern float rpts1[POINTS_MAX_LEN][2];
extern int rpts0_num, rpts1_num;
// 变换后左右边线+滤波
extern float rpts0b[POINTS_MAX_LEN][2];
extern float rpts1b[POINTS_MAX_LEN][2];
extern int rpts0b_num, rpts1b_num;
// 变换后左右边线+等距采样
extern float rpts0s[POINTS_MAX_LEN][2];
extern float rpts1s[POINTS_MAX_LEN][2];
extern int rpts0s_num, rpts1s_num;
// 左右边线局部角度变化率
extern float rpts0a[POINTS_MAX_LEN];
extern float rpts1a[POINTS_MAX_LEN];
extern int rpts0a_num, rpts1a_num;
// 左右边线局部角度变化率+非极大抑制
extern float rpts0an[POINTS_MAX_LEN];
extern float rpts1an[POINTS_MAX_LEN];
extern int rpts0an_num, rpts1an_num;
// 左/右中线
extern float rptsc0[POINTS_MAX_LEN][2];
extern float rptsc1[POINTS_MAX_LEN][2];
extern int rptsc0_num, rptsc1_num;
// 中线
extern float(*rpts)[2];
extern int rpts_num;
// 归一化中线
extern float rptsn[POINTS_MAX_LEN][2];
extern int rptsn_num;

// L角点-远
extern int Lpt0_far_rpts0s_id, Lpt1_far_rpts1s_id;
extern bool Lpt0_far_found, Lpt1_far_found;

// L角点
extern int Lpt0_rpts0s_id, Lpt1_rpts1s_id;
extern bool Lpt0_found, Lpt1_found;

//以下为十字控制寻远线操作,算法与常规寻线相同

extern bool far_Lpt0_found, far_Lpt1_found;
extern int far_Lpt0_rpts0s_id, far_Lpt1_rpts1s_id;
//原图远线左右边线
extern int far_ipts0[FAR_POINTS_MAX_LEN][2];
extern int far_ipts1[FAR_POINTS_MAX_LEN][2];
extern int far_ipts0_num, far_ipts1_num;
//变换后左右远边线
extern float far_rpts0[FAR_POINTS_MAX_LEN][2];
extern float far_rpts1[FAR_POINTS_MAX_LEN][2];
extern int far_rpts0_num, far_rpts1_num;
//变换后左右远边线+滤波
extern float far_rpts0b[FAR_POINTS_MAX_LEN][2];
extern float far_rpts1b[FAR_POINTS_MAX_LEN][2];
extern int far_rpts0b_num, far_rpts1b_num;
//变换后左右远边线+等距采样
extern float far_rpts0s[FAR_POINTS_MAX_LEN][2];
extern float far_rpts1s[FAR_POINTS_MAX_LEN][2];
extern int far_rpts0s_num, far_rpts1s_num;
// 左右远边线局部角度变化率
extern float far_rpts0a[FAR_POINTS_MAX_LEN];
extern float far_rpts1a[FAR_POINTS_MAX_LEN];
extern int far_rpts0a_num, far_rpts1a_num;
// 左右远边线局部角度变化率+非极大抑制
extern float far_rpts0an[FAR_POINTS_MAX_LEN];
extern float far_rpts1an[FAR_POINTS_MAX_LEN];
extern int far_rpts0an_num, far_rpts1an_num;

extern bool is_straight0, is_straight1;

enum track_type_e {
    TRACK_LEFT,
    TRACK_RIGHT,
};
extern enum track_type_e track_type;


extern double stdevLeft;    // 边缘方差
extern double stdevRight;   // 边缘方差

extern double Lradius;
extern double Rradius;
extern double Lradius_far;
extern double Rradius_far;

extern float radius;       // 圆弧半径

extern int buzzer;

// 原图左右边线(圆环)
extern int circle_ipts[POINTS_MAX_LEN][2];
extern int circle_ipts_num;

// 变换后左右边线+等距采样
extern float circle_rptss[POINTS_MAX_LEN][2];
extern int circle_rptss_num;
// L角点
extern int circle_Lpt_rptss_id;
extern bool circle_Lpt_found;

extern float danger_rpts[POINTS_MAX_LEN][2];
extern int danger_rpts_num;
extern float danger_rptsb[POINTS_MAX_LEN][2];
extern int danger_rptsb_num;
extern float danger_rptss[POINTS_MAX_LEN][2];
extern int danger_rptss_num;


extern int maxWhiteCOL;
extern int maxWhiteROW;

extern int far_x1;
extern int far_x2;
extern int far_y1;
extern int far_y2;
extern int far_x11;
extern int far_x22;
extern int far_y11;
extern int far_y22;

// 停车标志位
extern bool stop;

extern std::atomic<bool> AI_ENABLE;
extern bool DANGER_ENABLE;
extern bool RESCUE_ENABLE;
extern bool RACING_ENABLE;
extern bool RACING_FIND;
extern bool BRIDGE_ENABLE;
extern int rescue_state;
extern float mid_x;
extern float mid_y;

// 丢线场计数
extern int OUT_COUNT;

// 短直道场计数
extern int SHORT_COUNT;

// 急减速场计数
extern int SLOW_COUNT;

// 加速场计数
extern int FAST_COUNT;

// 速度
extern int SPEED_BRIDGE;
extern int SPEED_DANGER;
extern int SPEED_RACING;
extern int SPEED_RESCUE;

extern int SPEED_NORMAL;
extern int SPEED_FAST;
extern int SPEED_SLOW;
extern int SPEED_SLOWEST;
extern int SPEED_ADD;

// 圆环速度
extern int SPEED_CIRCLE_BEGIN;
extern int SPEED_CIRCLE_MIN;

extern int SPEED_CIRCLE_RUNNING;

// 偏差速度控制值abs(4500 - midAdd) > BIAS_SPEED_LIMIT后进入慢速控制
extern int BIAS_SPEED_LIMIT;

// 逆透视半径计算瞄点
extern int LR_num1;
extern int LR_num2;
extern int LR_num3;

extern int RR_num1;
extern int RR_num2;
extern int RR_num3;

extern int LR_F_num1;
extern int LR_F_num2;
extern int LR_F_num3;

extern int RR_F_num1;
extern int RR_F_num2;
extern int RR_F_num3;

extern double AIM_DISTANCE;

extern bool ai_debug;      // AI图像显示使能
extern bool bi_debug;      // 二值化图像显示使能
extern bool track_debug;   // 赛道线图像显示使能
extern bool nts_debug;     // 逆透视图像显示使能
// extern bool danger_debug;  // 危险区图像显示使能
extern bool saveImage;     // 存图使能

// // 速度控制变量(2024年4月28日改为结构体Speed)
// extern int speed_normal;
// extern int speed_fast;
// extern int speed_slow;
// extern int speed_add;

// 舵机偏差计算变量
extern double bias_p;
extern double bias_p_last;
extern double bias_d;
extern double ctr;
extern double midAdd;
extern float mid_x;
extern float mid_y;
#endif //! __MAIN_H__
