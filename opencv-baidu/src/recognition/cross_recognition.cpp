#include "cross_recognition.h"
#include "../src/main.h"
#include "../code/headfile.h"
#include "../code/camera_param.h"
#include "../code/imgproc.h"


enum cross_type_e cross_type = CROSS_NONE;

const char* cross_type_name[CROSS_NUM] = {
        "CROSS_NONE",
        "CROSS_BEGIN", "CROSS_IN",
        "CROSS_HALF",
        "CROSS_HALF_BEGIN",
        "CROSS_HALF_LEFT",
        "CROSS_HALF_RIGHT",
};

//// 编码器值，用于防止一些重复触发等。
//int64_t cross_encoder;

//十字远端左观点       十字远端右观点
bool far_Lpt0_found, far_Lpt1_found;
int far_Lpt0_rpts0s_id, far_Lpt1_rpts1s_id;

//↓↓↓↓↓#include "../code/camera_param.h"
//
// 
// 
//  double mapx[240][320];
//extern double mapy[240][320];

// 以下定义为十字寻远线设定
int far_ipts0[FAR_POINTS_MAX_LEN][2];
int far_ipts1[FAR_POINTS_MAX_LEN][2];
int far_ipts0_num, far_ipts1_num;

float far_rpts0[FAR_POINTS_MAX_LEN][2];
float far_rpts1[FAR_POINTS_MAX_LEN][2];
int far_rpts0_num, far_rpts1_num;

float far_rpts0b[FAR_POINTS_MAX_LEN][2];
float far_rpts1b[FAR_POINTS_MAX_LEN][2];
int far_rpts0b_num, far_rpts1b_num;

float far_rpts0s[FAR_POINTS_MAX_LEN][2];
float far_rpts1s[FAR_POINTS_MAX_LEN][2];
int far_rpts0s_num, far_rpts1s_num;

float far_rpts0a[FAR_POINTS_MAX_LEN];
float far_rpts1a[FAR_POINTS_MAX_LEN];
int far_rpts0a_num, far_rpts1a_num;

float far_rpts0an[FAR_POINTS_MAX_LEN];
float far_rpts1an[FAR_POINTS_MAX_LEN];
int far_rpts0an_num, far_rpts1an_num;

int not_have_line = 0;

int far_x1 = 76, far_x2 = 244, far_y1, far_y2;
int far_x11 = 76, far_x22 = 244, far_y11, far_y22;

//双L角点,切十字模式
void check_cross() {
    bool Xfound = Lpt0_found && Lpt1_found;
    if (Xfound) {
        // 左L点在右L点上方，使用右L点
        if (rpts0s[clip(Lpt0_rpts0s_id, 0, rpts0s_num - 1)][1] < rpts0s[clip(Lpt0_rpts0s_id, 0, rpts0s_num - 1)][1]) {
            Lpt0_found = false;
            check_Half_right();
        }
        else {
            Lpt1_found = false;
            check_Half_left();
        }
    }
    //if (cross_type == CROSS_NONE && Xfound) cross_type = CROSS_BEGIN;
}

void check_Half() {
    if (Lpt0_found && rpts0s_num > rpts1s_num) {
        check_Half_left();
    }
    if (Lpt1_found && rpts1s_num > rpts0s_num) {
        check_Half_right();
    }
}
void check_Half_left() {
    cross_farline_L();
    //// L 点二次检查
    //if (far_Lpt0_found && Lpt0_found) {
    //    float dx = far_rpts0s[far_Lpt0_rpts0s_id][0] - rpts0s[Lpt0_rpts0s_id][0];
    //    float dy = far_rpts0s[far_Lpt0_rpts0s_id][1] - rpts0s[Lpt0_rpts0s_id][1];
    //    float dn = sqrtf(dx * dx + dy * dy);
    //    if (fabs(dn - 0.35 * pixel_per_meter) > 0.35 * pixel_per_meter) {
    //        COUT1("L 点二次检查 LEFT false");
    //        far_Lpt0_found = false;
    //    }
    //}

    if (far_Lpt0_found && cross_type == CROSS_NONE && !is_straight1 && Lpt0_rpts0s_id < 0.7 / sample_dist) {
        rpts0s_num = Lpt0_rpts0s_id;
        cross_type = CROSS_HALF_LEFT;
    }

}

void check_Half_right() {
    cross_farline_R();
    //// L 点二次检查
    //if (far_Lpt1_found && Lpt1_found) {
    //    float dx = far_rpts1s[far_Lpt1_rpts1s_id][0] - rpts1s[Lpt1_rpts1s_id][0];
    //    float dy = far_rpts1s[far_Lpt1_rpts1s_id][1] - rpts1s[Lpt1_rpts1s_id][1];
    //    float dn = sqrtf(dx * dx + dy * dy);
    //    if (fabs(dn - 0.35 * pixel_per_meter) > 0.35 * pixel_per_meter) {
    //        COUT1("L 点二次检查 RIGHT false");
    //        far_Lpt1_found = false;
    //    }
    //}

    if (far_Lpt1_found && cross_type == CROSS_NONE && !is_straight0 && Lpt1_rpts1s_id < 0.7 / sample_dist) {
        rpts1s_num = Lpt1_rpts1s_id;
        cross_type = CROSS_HALF_RIGHT;
    }
}

int cross_count = 0;
void run_cross() {
    bool Xfound = Lpt0_found && Lpt1_found;
    float Lpt0y = rpts0s[Lpt0_rpts0s_id][1];
    float Lpt1y = rpts1s[Lpt1_rpts1s_id][1];

    //检测到十字，先按照近线走
    if (cross_type == CROSS_BEGIN) {
        if (cross_count++ > 30) {
            cross_count = 0;
            cross_type = CROSS_NONE;
        }

        if (Lpt0_found) {
            rptsc0_num = rpts0s_num = Lpt0_rpts0s_id;
        }
        if (Lpt1_found) {
            rptsc1_num = rpts1s_num = Lpt1_rpts1s_id;
        }
        //近角点过少，进入远线控制
        //COUT2(0.1 / sample_dist, Lpt0_rpts0s_id);
        if ((Xfound &&
            //拐点距离车轮25厘米时，进入远线控制
            (Lpt0_rpts0s_id < 0.7 / sample_dist && Lpt1_rpts1s_id < 0.7 / sample_dist))
            /* || (rpts1_num <30 && rpts0_num<30)*/) {
            bias_i = 0;
            cross_type = CROSS_IN;
        }
    }
    //远线控制进十字,begin_y渐变靠近防丢线
    else if (cross_type == CROSS_IN) {
        if (cross_count++ > 50) {
            cross_count = 0;
            cross_type = CROSS_NONE;
        }
        //寻远线,算法与近线相同
        cross_farline();

        if (rpts1s_num < 5 && rpts0s_num < 5) { not_have_line++; }
        if (not_have_line > 2 && rpts1s_num > 20 && rpts0s_num > 20) {
            cross_type = CROSS_NONE;
            not_have_line = 0;
        }
        if (far_Lpt1_found) { track_type = TRACK_RIGHT; }
        else if (far_Lpt0_found) { track_type = TRACK_LEFT; }
        else if (not_have_line > 0 && rpts1s_num < 5) { track_type = TRACK_RIGHT; }
        else if (not_have_line > 0 && rpts0s_num < 5) { track_type = TRACK_LEFT; }

        if (track_type == TRACK_RIGHT && far_rpts1s[far_Lpt1_rpts1s_id][0] > 240) {
            track_type = TRACK_LEFT;
        }
    }
    else if (cross_type == CROSS_HALF_LEFT) {
        cross_farline_L();
        // aim_distance = 0.45;
        track_type = TRACK_LEFT;
        if (rpts0s_num < 5) { not_have_line++; }
        if (not_have_line > 2 & rpts1s_num > 20 && rpts0s_num > 20) {
            cross_type = CROSS_NONE;
            not_have_line = 0;
        }

    }
    else if (cross_type == CROSS_HALF_RIGHT) {
        cross_farline_R();
        // aim_distance = 0.45;
        track_type = TRACK_RIGHT;
        if (rpts1s_num < 5) { not_have_line++; }
        if (not_have_line > 2 & rpts1s_num > 20 && rpts0s_num > 20) {
            cross_type = CROSS_NONE;
            not_have_line = 0;
        }
    }

}

// 绘制十字模式下的调试图像
void draw_cross() {
    if (cross_type == CROSS_IN && line_show_sample) {
        for (int i = 0; i < far_rpts0s_num; i++) {
            AT_IMAGE(&img_line, clip(far_rpts0s[i][0], 0, img_line.width - 1), clip(far_rpts0s[i][1], 0, img_line.height - 1)) = 255;
        }
        for (int i = 0; i < far_rpts1s_num; i++) {
            AT_IMAGE(&img_line, clip(far_rpts1s[i][0], 0, img_line.width - 1), clip(far_rpts1s[i][1], 0, img_line.height - 1)) = 255;
        }

        //十字远端左观点
        if (far_Lpt0_found) {
            draw_o(&img_line, far_rpts0s[far_Lpt0_rpts0s_id][0], far_rpts0s[far_Lpt0_rpts0s_id][1], 3, 255);
        }
        //十字远端右观点
        if (far_Lpt1_found) {
            draw_o(&img_line, far_rpts1s[far_Lpt1_rpts1s_id][0], far_rpts1s[far_Lpt1_rpts1s_id][1], 3, 255);
        }

        draw_o(&img_line, clip(mapx[(int)begin_y][far_x1], 0, img_line.width - 1), clip(mapy[(int)begin_y][far_x1], 0, img_line.height - 1), 5, 255);
        draw_o(&img_line, clip(mapx[(int)begin_y][far_x2], 0, img_line.width - 1), clip(mapy[(int)begin_y][far_x1], 0, img_line.height - 1), 10, 255);
        draw_o(&img_line, clip(mapx[far_y1][far_x1], 0, img_line.width - 1), clip(mapy[far_y1][far_x1], 0, img_line.height - 1), 15, 255);
        draw_o(&img_line, clip(mapx[far_y2][far_x2], 0, img_line.width - 1), clip(mapy[far_y2][far_x2], 0, img_line.height - 1), 20, 255);

        /*
        for(int y1=begin_y; y1>far_y1; y1--){
            AT_IMAGE(&img_raw, far_x1, y1) = 128;
        }
        for(int y2=begin_y; y2>far_y2; y2--){
            AT_IMAGE(&img_raw, far_x2, y2) = 128;
        }
        */
    }
    else if (cross_type == CROSS_HALF_LEFT) {
        for (int i = 0; i < far_rpts0s_num; i++) {
            AT_IMAGE(&img_line, clip(far_rpts0s[i][0], 0, img_line.width - 1), clip(far_rpts0s[i][1], 0, img_line.height - 1)) = 255;
        }

        //十字远端左观点
        if (far_Lpt0_found) {
            draw_o(&img_line, far_rpts0s[far_Lpt0_rpts0s_id][0], far_rpts0s[far_Lpt0_rpts0s_id][1], 3, 255);
        }

        draw_o(&img_line, clip(mapx[(int)begin_y][far_x11], 0, img_line.width - 1), clip(mapy[(int)begin_y][far_x11], 0, img_line.height - 1), 5, 255);
        draw_o(&img_line, clip(mapx[far_y11][far_x11], 0, img_line.width - 1), clip(mapy[far_y11][far_x11], 0, img_line.height - 1), 15, 255);
    }
    else if (cross_type == CROSS_HALF_RIGHT) {
        for (int i = 0; i < far_rpts1s_num; i++) {
            AT_IMAGE(&img_line, clip(far_rpts1s[i][0], 0, img_line.width - 1), clip(far_rpts1s[i][1], 0, img_line.height - 1)) = 255;
        }

        //十字远端右观点
        if (far_Lpt1_found) {
            draw_o(&img_line, far_rpts1s[far_Lpt1_rpts1s_id][0], far_rpts1s[far_Lpt1_rpts1s_id][1], 3, 255);
        }

        draw_o(&img_line, clip(mapx[(int)begin_y][far_x22], 0, img_line.width - 1), clip(mapy[(int)begin_y][far_x11], 0, img_line.height - 1), 10, 255);
        draw_o(&img_line, clip(mapx[far_y22][far_x22], 0, img_line.width - 1), clip(mapy[far_y22][far_x22], 0, img_line.height - 1), 20, 255);

    }
}

void cross_farline() {
    int cross_width = 4;
    //    far_x1 = cross_width, far_x2 = img_raw.width -cross_width;
    far_y1 = 0, far_y2 = 0;


    int x1 = img_raw.width / 2 - begin_x / 2, y1 = begin_y;
    bool white_found = false;
    far_ipts0_num = sizeof(far_ipts0) / sizeof(far_ipts0[0]);

    // if (Lpt0_found) {
    //     int p[2];
    //     map_inv(rpts0s[Lpt0_rpts0s_id], p);
    //     far_x1 = p[0] - 20;
    // }
    for (; y1 > 0; y1--) {
        //先黑后白，先找white
        if (AT_IMAGE(&img_raw, far_x1, y1) >= thres) { white_found = true; }
        if (AT_IMAGE(&img_raw, far_x1, y1) < thres && (white_found || far_x1 == cross_width)) {
            far_y1 = y1;
            break;
        }
    }

    //从找到角点位置开始寻找
    if (AT_IMAGE(&img_raw, far_x1, far_y1 + 1) >= thres)
        findline_lefthand_adaptive(&img_raw, block_size, clip_value, far_x1, far_y1 + 1, far_ipts0, &far_ipts0_num);
    else far_ipts0_num = 0;

    int x2 = img_raw.width / 2 + begin_x / 2, y2 = begin_y;
    white_found = false;
    far_ipts1_num = sizeof(far_ipts1) / sizeof(far_ipts1[0]);

    // if (Lpt1_found) {
    //     int p[2];
    //     map_inv(rpts1s[Lpt1_rpts1s_id], p);
    //     far_x2 = p[0] + 20;
    // }
    for (; y2 > 0; y2--) {
        //先黑后白，先找white
        if (AT_IMAGE(&img_raw, far_x2, y2) >= thres) { white_found = true; }
        if (AT_IMAGE(&img_raw, far_x2, y2) < thres && (white_found || far_x2 == img_raw.width - cross_width)) {
            far_y2 = y2;
            break;
        }
    }

    //从找到角点位置开始寻找
    if (AT_IMAGE(&img_raw, far_x2, far_y2 + 1) >= thres)
        findline_righthand_adaptive(&img_raw, block_size, clip_value, far_x2, far_y2 + 1, far_ipts1, &far_ipts1_num);
    else far_ipts1_num = 0;


    // 去畸变+透视变换
    for (int i = 0; i < far_ipts0_num; i++) {
        far_rpts0[i][0] = mapx[far_ipts0[i][1]][far_ipts0[i][0]];
        far_rpts0[i][1] = mapy[far_ipts0[i][1]][far_ipts0[i][0]];
    }
    far_rpts0_num = far_ipts0_num;
    for (int i = 0; i < far_ipts1_num; i++) {
        far_rpts1[i][0] = mapx[far_ipts1[i][1]][far_ipts1[i][0]];
        far_rpts1[i][1] = mapy[far_ipts1[i][1]][far_ipts1[i][0]];
    }
    far_rpts1_num = far_ipts1_num;


    // 边线滤波
    blur_points(far_rpts0, far_rpts0_num, far_rpts0b, (int)round(line_blur_kernel));
    far_rpts0b_num = far_rpts0_num;
    blur_points(far_rpts1, far_rpts1_num, far_rpts1b, (int)round(line_blur_kernel));
    far_rpts1b_num = far_rpts1_num;

    // 边线等距采样
    far_rpts0s_num = sizeof(far_rpts0s) / sizeof(far_rpts0s[0]);
    resample_points(far_rpts0b, far_rpts0b_num, far_rpts0s, &far_rpts0s_num, sample_dist * pixel_per_meter);
    far_rpts1s_num = sizeof(far_rpts1s) / sizeof(far_rpts1s[0]);
    resample_points(far_rpts1b, far_rpts1b_num, far_rpts1s, &far_rpts1s_num, sample_dist * pixel_per_meter);

    // 边线局部角度变化率
    local_angle_points(far_rpts0s, far_rpts0s_num, far_rpts0a, (int)round(angle_dist / sample_dist));
    far_rpts0a_num = far_rpts0s_num;
    local_angle_points(far_rpts1s, far_rpts1s_num, far_rpts1a, (int)round(angle_dist / sample_dist));
    far_rpts1a_num = far_rpts1s_num;

    // 角度变化率非极大抑制
    nms_angle(far_rpts0a, far_rpts0a_num, far_rpts0an, (int)round(angle_dist / sample_dist) * 2 + 1);
    far_rpts0an_num = far_rpts0a_num;
    nms_angle(far_rpts1a, far_rpts1a_num, far_rpts1an, (int)round(angle_dist / sample_dist) * 2 + 1);
    far_rpts1an_num = far_rpts1a_num;

    // 找远线上的L角点
    far_Lpt0_found = far_Lpt1_found = false;
    for (int i = 0; i < MIN(far_rpts0s_num, 80); i++) {
        if (far_rpts0an[i] == 0) continue;
        int im1 = clip(i - (int)round(angle_dist / sample_dist), 0, far_rpts0s_num - 1);
        int ip1 = clip(i + (int)round(angle_dist / sample_dist), 0, far_rpts0s_num - 1);
        float conf = fabs(far_rpts0a[i]) - (fabs(far_rpts0a[im1]) + fabs(far_rpts0a[ip1])) / 2;
        if (70. / 180. * PI < conf && conf < 110. / 180. * PI && i < 100) {
            far_Lpt0_rpts0s_id = i;
            far_Lpt0_found = true;
            break;
        }
    }
    for (int i = 0; i < MIN(far_rpts1s_num, 80); i++) {
        if (far_rpts1an[i] == 0) continue;
        int im1 = clip(i - (int)round(angle_dist / sample_dist), 0, far_rpts1s_num - 1);
        int ip1 = clip(i + (int)round(angle_dist / sample_dist), 0, far_rpts1s_num - 1);
        float conf = fabs(far_rpts1a[i]) - (fabs(far_rpts1a[im1]) + fabs(far_rpts1a[ip1])) / 2;

        if (70. / 180. * PI < conf && conf < 110. / 180. * PI && i < 100) {
            far_Lpt1_rpts1s_id = i;
            far_Lpt1_found = true;
            break;
        }
    }
}


void cross_farline_L() {
    int Lpt0_P[2] = { 76, begin_y };

    if (Lpt0_found && rpts0s_num >= 3) {
        int p[2];
        map_inv(rpts0s[clip(Lpt0_rpts0s_id, 0, rpts0s_num - 1)], p);
        p[0] -= 20;

        Lpt0_P[0] = p[0];
        Lpt0_P[1] = p[1];
    }
    else if (rpts0s_num < 2) {
        Lpt0_P[0] = 76;
        Lpt0_P[1] = begin_y;
    }

    if (Lpt1_found) {
        rptsc1_num = rpts1s_num = Lpt1_rpts1s_id - 2;
    }

    int cross_width = 4;
    bool white_found = false;
    far_ipts0_num = sizeof(far_ipts0) / sizeof(far_ipts0[0]);

    int y1 = Lpt0_P[1];
    far_x11 = Lpt0_P[0];
    far_y11 = 0;


    for (; y1 > 0; y1--) {
        //先黑后白，先找white
        if (AT_IMAGE(&img_raw, far_x11, y1) >= thres) { white_found = true; }
        if (AT_IMAGE(&img_raw, far_x11, y1) < thres && white_found) {
            far_y11 = y1;
            break;
        }
    }

    //从找到角点位置开始寻找
    if (AT_IMAGE(&img_raw, far_x11, far_y11 + 1) >= thres)
        findline_lefthand_adaptive(&img_raw, block_size, clip_value, far_x11, far_y11 + 1, far_ipts0, &far_ipts0_num);
    else far_ipts0_num = 0;

    int x2 = img_raw.width / 2 + begin_x / 2, y2 = begin_y;
    white_found = false;
    far_ipts1_num = sizeof(far_ipts1) / sizeof(far_ipts1[0]);


    // 去畸变+透视变换
    for (int i = 0; i < far_ipts0_num; i++) {
        far_rpts0[i][0] = mapx[far_ipts0[i][1]][far_ipts0[i][0]];
        far_rpts0[i][1] = mapy[far_ipts0[i][1]][far_ipts0[i][0]];
    }
    far_rpts0_num = far_ipts0_num;

    // 边线滤波
    blur_points(far_rpts0, far_rpts0_num, far_rpts0b, (int)round(line_blur_kernel));
    far_rpts0b_num = far_rpts0_num;

    // 边线等距采样
    far_rpts0s_num = sizeof(far_rpts0s) / sizeof(far_rpts0s[0]);
    resample_points(far_rpts0b, far_rpts0b_num, far_rpts0s, &far_rpts0s_num, sample_dist * pixel_per_meter);

    // 边线局部角度变化率
    local_angle_points(far_rpts0s, far_rpts0s_num, far_rpts0a, (int)round(angle_dist / sample_dist));
    far_rpts0a_num = far_rpts0s_num;

    // 角度变化率非极大抑制
    nms_angle(far_rpts0a, far_rpts0a_num, far_rpts0an, (int)round(angle_dist / sample_dist) * 2 + 1);
    far_rpts0an_num = far_rpts0a_num;

    // 找远线上的L角点
    far_Lpt0_found = far_Lpt1_found = false;
    for (int i = 0; i < MIN(far_rpts0s_num, 80); i++) {
        if (far_rpts0an[i] == 0) continue;
        int im1 = clip(i - (int)round(angle_dist / sample_dist), 0, far_rpts0s_num - 1);
        int ip1 = clip(i + (int)round(angle_dist / sample_dist), 0, far_rpts0s_num - 1);
        float conf = fabs(far_rpts0a[i]) - (fabs(far_rpts0a[im1]) + fabs(far_rpts0a[ip1])) / 2;
        if (70. / 180. * PI < conf && conf < 110. / 180. * PI && i < 100) {
            far_Lpt0_rpts0s_id = i;
            far_Lpt0_found = true;
            break;
        }
    }
}

void cross_farline_R() {
    int Lpt1_P[2] = { 244, begin_y };

    if (Lpt1_found && rpts1s_num >= 3) {
        int p[2];
        map_inv(rpts1s[clip(Lpt1_rpts1s_id, 0, rpts1s_num - 1)], p);
        p[0] += 20;

        Lpt1_P[0] = p[0];
        Lpt1_P[1] = p[1];
    }
    else {
        Lpt1_P[0] = 244;
        Lpt1_P[1] = begin_y;
    }

    if (Lpt0_found) {
        rptsc0_num = rpts0s_num = Lpt0_rpts0s_id - 2;
    }


    int cross_width = 4;
    far_y1 = 0, far_y2 = 0;

    bool white_found = false;
    far_ipts1_num = sizeof(far_ipts1) / sizeof(far_ipts1[0]);

    int y2 = Lpt1_P[1];
    far_x22 = Lpt1_P[0];
    far_y22 = 0;

    for (; y2 > 0; y2--) {
        //先黑后白，先找white
        if (AT_IMAGE(&img_raw, far_x22, y2) >= thres) { white_found = true; }
        if (AT_IMAGE(&img_raw, far_x22, y2) < thres && (white_found || far_x22 == img_raw.width - cross_width)) {
            far_y22 = y2;
            break;
        }
    }

    //从找到角点位置开始寻找
    if (AT_IMAGE(&img_raw, far_x22, far_y22 + 1) >= thres)
        findline_righthand_adaptive(&img_raw, block_size, clip_value, far_x22, far_y22 + 1, far_ipts1, &far_ipts1_num);
    else far_ipts1_num = 0;


    // 去畸变+透视变换
    for (int i = 0; i < far_ipts1_num; i++) {
        far_rpts1[i][0] = mapx[far_ipts1[i][1]][far_ipts1[i][0]];
        far_rpts1[i][1] = mapy[far_ipts1[i][1]][far_ipts1[i][0]];
    }
    far_rpts1_num = far_ipts1_num;


    // 边线滤波
    blur_points(far_rpts1, far_rpts1_num, far_rpts1b, (int)round(line_blur_kernel));
    far_rpts1b_num = far_rpts1_num;

    // 边线等距采样
    far_rpts1s_num = sizeof(far_rpts1s) / sizeof(far_rpts1s[0]);
    resample_points(far_rpts1b, far_rpts1b_num, far_rpts1s, &far_rpts1s_num, sample_dist * pixel_per_meter);

    // 边线局部角度变化率
    local_angle_points(far_rpts1s, far_rpts1s_num, far_rpts1a, (int)round(angle_dist / sample_dist));
    far_rpts1a_num = far_rpts1s_num;

    // 角度变化率非极大抑制
    nms_angle(far_rpts1a, far_rpts1a_num, far_rpts1an, (int)round(angle_dist / sample_dist) * 2 + 1);
    far_rpts1an_num = far_rpts1a_num;

    // 找远线上的L角点
    for (int i = 0; i < MIN(far_rpts1s_num, 80); i++) {
        if (far_rpts1an[i] == 0) continue;
        int im1 = clip(i - (int)round(angle_dist / sample_dist), 0, far_rpts1s_num - 1);
        int ip1 = clip(i + (int)round(angle_dist / sample_dist), 0, far_rpts1s_num - 1);
        float conf = fabs(far_rpts1a[i]) - (fabs(far_rpts1a[im1]) + fabs(far_rpts1a[ip1])) / 2;

        if (70. / 180. * PI < conf && conf < 110. / 180. * PI && i < 100) {
            far_Lpt1_rpts1s_id = i;
            far_Lpt1_found = true;
            break;
        }
    }
}