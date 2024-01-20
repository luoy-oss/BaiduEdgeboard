#include "cross_recognition.h"
#include "../src/main.h"
#include "../code/headfile.h"
#include "../code/camera_param.h"
#include "../code/imgproc.h"


enum cross_type_e cross_type = CROSS_NONE;

const char* cross_type_name[CROSS_NUM] = {
        "CROSS_NONE",
        "CROSS_BEGIN", "CROSS_IN",
};

//// ������ֵ�����ڷ�ֹһЩ�ظ������ȡ�
//int64_t cross_encoder;

//ʮ��Զ����۵�       ʮ��Զ���ҹ۵�
bool far_Lpt0_found, far_Lpt1_found;
int far_Lpt0_rpts0s_id, far_Lpt1_rpts1s_id;

//����������#include "../code/camera_param.h"
//
// 
// 
//  double mapx[240][320];
//extern double mapy[240][320];

// ���¶���Ϊʮ��ѰԶ���趨
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

int far_x1 = 86 / IMAGESCALE, far_x2 = 260 / IMAGESCALE, far_y1, far_y2;
int maxWhiteCOL = 160;
int maxWhiteROW = 0;

//˫L�ǵ�,��ʮ��ģʽ
void check_cross() {
    bool Xfound = Lpt0_found && Lpt1_found;
    if (cross_type == CROSS_NONE && Xfound) cross_type = CROSS_BEGIN;
}

int cross_count = 0;
void run_cross() {
    bool Xfound = Lpt0_found && Lpt1_found;
    //    int64_t current_encoder = get_total_encoder();
    float Lpt0y = rpts0s[Lpt0_rpts0s_id][1];
    float Lpt1y = rpts1s[Lpt1_rpts1s_id][1];

    maxWhiteCOL = 160;
    maxWhiteROW = 0;
    // Ѱ�������
    if (cross_type != CROSS_NONE) {
        int maxWhiteCount = 0;
        for (int c = 0; c < COLSIMAGE / IMAGESCALE; c++) {
            int r = begin_y;
            for (; r >= 0; r--) if (AT_IMAGE(&img_raw, c, r) != 255) break;

            if (begin_y - r > maxWhiteCount) {
                maxWhiteCount = begin_y - r;
                maxWhiteCOL = c;
                maxWhiteROW = r;
            }
        }
    }

    //��⵽ʮ�֣��Ȱ��ս�����
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
        //���ǵ���٣�����Զ�߿���
        //COUT2(0.1 / sample_dist, Lpt0_rpts0s_id);
        if ((Xfound &&
            //�յ���복��25����ʱ������Զ�߿���
            (Lpt0_rpts0s_id < 0.7 / sample_dist && Lpt1_rpts1s_id < 0.7 / sample_dist))
            /* || (rpts1_num <30 && rpts0_num<30)*/) {
            bias_i = 0;
            cross_type = CROSS_IN;
        }
    }
    //Զ�߿��ƽ�ʮ��,begin_y���俿��������
    else if (cross_type == CROSS_IN) {
        //if (cross_count++ > 60) {
        //    cross_count = 0;
        //    cross_type = CROSS_NONE;
        //}
        ////ѰԶ��,�㷨�������ͬ
        //cross_farline();

        //if (rpts1s_num < 5 && rpts0s_num < 5) { not_have_line++; }
        //if (not_have_line > 2 && rpts1s_num > 20 && rpts0s_num > 20) {
        //    cross_type = CROSS_NONE;
        //    not_have_line = 0;
        //}
        //if (far_Lpt1_found) { track_type = TRACK_RIGHT; }
        //else if (far_Lpt0_found) { track_type = TRACK_LEFT; }
        //else if (not_have_line > 0 && rpts1s_num < 5) { track_type = TRACK_RIGHT; }
        //else if (not_have_line > 0 && rpts0s_num < 5) { track_type = TRACK_LEFT; }

        //if (track_type == TRACK_RIGHT && far_rpts1s[far_Lpt1_rpts1s_id][0] > 240) {
        //    track_type = TRACK_LEFT;
        //}

        if (cross_count++ > 60) {
            cross_count = 0;
            cross_type = CROSS_NONE;
        }
        if (rpts1s_num < 5 && rpts0s_num < 5) { not_have_line++; }
        if (not_have_line > 2 && rpts1s_num > 20 && rpts0s_num > 20) {
            cross_type = CROSS_NONE;
            not_have_line = 0;
        }
        /*if (far_Lpt1_found) { track_type = TRACK_RIGHT; }
        else if (far_Lpt0_found) { track_type = TRACK_LEFT; }
        else if (not_have_line > 0 && rpts1s_num < 5) { track_type = TRACK_RIGHT; }
        else if (not_have_line > 0 && rpts0s_num < 5) { track_type = TRACK_LEFT; }

        if (track_type == TRACK_RIGHT && far_rpts1s[far_Lpt1_rpts1s_id][0] > 240) {
            track_type = TRACK_LEFT;
        }*/

    }
}

// ����ʮ��ģʽ�µĵ���ͼ��
void draw_cross() {
    if (cross_type == CROSS_IN && line_show_sample) {
        for (int i = 0; i < far_rpts0s_num; i++) {
            AT_IMAGE(&img_line, clip(far_rpts0s[i][0], 0, img_line.width - 1), clip(far_rpts0s[i][1], 0, img_line.height - 1)) = 255;
        }
        for (int i = 0; i < far_rpts1s_num; i++) {
            AT_IMAGE(&img_line, clip(far_rpts1s[i][0], 0, img_line.width - 1), clip(far_rpts1s[i][1], 0, img_line.height - 1)) = 255;
        }

        //ʮ��Զ����۵�
        if (far_Lpt0_found) {
            draw_o(&img_line, far_rpts0s[far_Lpt0_rpts0s_id][0], far_rpts0s[far_Lpt0_rpts0s_id][1], 3 / IMAGESCALE, 255);
        }
        //ʮ��Զ���ҹ۵�
        if (far_Lpt1_found) {
            draw_o(&img_line, far_rpts1s[far_Lpt1_rpts1s_id][0], far_rpts1s[far_Lpt1_rpts1s_id][1], 3 / IMAGESCALE, 255);
        }

        draw_o(&img_line, clip(mapx[(int)begin_y][far_x1], 0, img_line.width - 1), clip(mapy[(int)begin_y][far_x1], 0, img_line.height - 1), 5 / IMAGESCALE, 255);
        draw_o(&img_line, clip(mapx[(int)begin_y][far_x2], 0, img_line.width - 1), clip(mapy[(int)begin_y][far_x1], 0, img_line.height - 1), 10 / IMAGESCALE, 255);
        draw_o(&img_line, clip(mapx[far_y1][far_x1], 0, img_line.width - 1), clip(mapy[far_y1][far_x1], 0, img_line.height - 1), 15 / IMAGESCALE, 255);
        draw_o(&img_line, clip(mapx[far_y2][far_x2], 0, img_line.width - 1), clip(mapy[far_y2][far_x2], 0, img_line.height - 1), 20 / IMAGESCALE, 255);

        /*
        for(int y1=begin_y; y1>far_y1; y1--){
            AT_IMAGE(&img_raw, far_x1, y1) = 128;
        }
        for(int y2=begin_y; y2>far_y2; y2--){
            AT_IMAGE(&img_raw, far_x2, y2) = 128;
        }
        */
    }
}

void cross_farline() {
    int cross_width = 4;
    //    far_x1 = cross_width, far_x2 = img_raw.width -cross_width;
    far_y1 = 0, far_y2 = 0;


    int x1 = img_raw.width / 2 - begin_x / 2, y1 = begin_y;
    bool white_found = false;
    far_ipts0_num = sizeof(far_ipts0) / sizeof(far_ipts0[0]);

    //��begin_y�������Һ���
//    for(;x1>cross_width*2; x1--) 
//    {
//      if(AT_IMAGE(&img_raw, x1-1, y1) < low_thres) {
//        far_x1 = x1 - cross_width;
//        break;
//      }   
//    }
    //ȫ��  far_x1 = 0,�ӱ߽���
    for (; y1 > 0; y1--) {
        //�Ⱥں�ף�����white
        if (AT_IMAGE(&img_raw, far_x1, y1) >= thres) { white_found = true; }
        if (AT_IMAGE(&img_raw, far_x1, y1) < thres && (white_found || far_x1 == cross_width)) {
            far_y1 = y1;
            break;
        }
    }

    //���ҵ��ǵ�λ�ÿ�ʼѰ��
    if (AT_IMAGE(&img_raw, far_x1, far_y1 + 1) >= thres)
        findline_lefthand_adaptive(&img_raw, block_size, clip_value, far_x1, far_y1 + 1, far_ipts0, &far_ipts0_num);
    else far_ipts0_num = 0;

    int x2 = img_raw.width / 2 + begin_x / 2, y2 = begin_y;
    white_found = false;
    far_ipts1_num = sizeof(far_ipts1) / sizeof(far_ipts1[0]);

    //��begin_y�������Һ���
//    for(;x2<img_raw.width-cross_width*2; x2++) 
//    {
//      if(AT_IMAGE(&img_raw, x2+1, y2) < low_thres) {
//        far_x2 = x2 + cross_width;
//        break;
//      }   
//    }
    //ȫ��  far_x2 = 0,�ӱ߽���
    for (; y2 > 0; y2--) {
        //�Ⱥں�ף�����white
        if (AT_IMAGE(&img_raw, far_x2, y2) >= thres) { white_found = true; }
        if (AT_IMAGE(&img_raw, far_x2, y2) < thres && (white_found || far_x2 == img_raw.width - cross_width)) {
            far_y2 = y2;
            break;
        }
    }

    //���ҵ��ǵ�λ�ÿ�ʼѰ��
    if (AT_IMAGE(&img_raw, far_x2, far_y2 + 1) >= thres)
        findline_righthand_adaptive(&img_raw, block_size, clip_value, far_x2, far_y2 + 1, far_ipts1, &far_ipts1_num);
    else far_ipts1_num = 0;


    // ȥ����+͸�ӱ任
    for (int i = 0; i < far_ipts0_num; i++) {
        far_rpts0[i][0] = mapx[far_ipts0[i][1] * IMAGESCALE][far_ipts0[i][0] * IMAGESCALE] / IMAGESCALE;
        far_rpts0[i][1] = mapy[far_ipts0[i][1] * IMAGESCALE][far_ipts0[i][0] * IMAGESCALE] / IMAGESCALE;
    }
    far_rpts0_num = far_ipts0_num;
    for (int i = 0; i < far_ipts1_num; i++) {
        far_rpts1[i][0] = mapx[far_ipts1[i][1] * IMAGESCALE][far_ipts1[i][0] * IMAGESCALE] / IMAGESCALE;
        far_rpts1[i][1] = mapy[far_ipts1[i][1] * IMAGESCALE][far_ipts1[i][0] * IMAGESCALE] / IMAGESCALE;
    }
    far_rpts1_num = far_ipts1_num;


    // �����˲�
    blur_points(far_rpts0, far_rpts0_num, far_rpts0b, (int)round(line_blur_kernel));
    far_rpts0b_num = far_rpts0_num;
    blur_points(far_rpts1, far_rpts1_num, far_rpts1b, (int)round(line_blur_kernel));
    far_rpts1b_num = far_rpts1_num;

    // ���ߵȾ����
    far_rpts0s_num = sizeof(far_rpts0s) / sizeof(far_rpts0s[0]);
    resample_points(far_rpts0b, far_rpts0b_num, far_rpts0s, &far_rpts0s_num, 1.5 * sample_dist * pixel_per_meter);
    far_rpts1s_num = sizeof(far_rpts1s) / sizeof(far_rpts1s[0]);
    resample_points(far_rpts1b, far_rpts1b_num, far_rpts1s, &far_rpts1s_num, 1.5 * sample_dist * pixel_per_meter);

    // ���߾ֲ��Ƕȱ仯��
    local_angle_points(far_rpts0s, far_rpts0s_num, far_rpts0a, (int)round(angle_dist / sample_dist));
    far_rpts0a_num = far_rpts0s_num;
    local_angle_points(far_rpts1s, far_rpts1s_num, far_rpts1a, (int)round(angle_dist / sample_dist));
    far_rpts1a_num = far_rpts1s_num;

    // �Ƕȱ仯�ʷǼ�������
    nms_angle(far_rpts0a, far_rpts0a_num, far_rpts0an, (int)round(angle_dist / sample_dist) * 2 + 1);
    far_rpts0an_num = far_rpts0a_num;
    nms_angle(far_rpts1a, far_rpts1a_num, far_rpts1an, (int)round(angle_dist / sample_dist) * 2 + 1);
    far_rpts1an_num = far_rpts1a_num;

    // ��Զ���ϵ�L�ǵ�
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
