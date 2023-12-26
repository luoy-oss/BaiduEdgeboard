#include "track_recognition.h"
#include "../src/main.h"
#include "../code/headfile.h"
#include "../code/camera_param.h"
#include "../code/imgproc.h"

// ԭͼ���ұ���
int ipts0[POINTS_MAX_LEN][2];
int ipts1[POINTS_MAX_LEN][2];
int ipts0_num, ipts1_num;
// �任�����ұ���
float rpts0[POINTS_MAX_LEN][2];
float rpts1[POINTS_MAX_LEN][2];
int rpts0_num, rpts1_num;
// �任�����ұ���+�˲�
float rpts0b[POINTS_MAX_LEN][2];
float rpts1b[POINTS_MAX_LEN][2];
int rpts0b_num, rpts1b_num;
// �任�����ұ���+�Ⱦ����
float rpts0s[POINTS_MAX_LEN][2];
float rpts1s[POINTS_MAX_LEN][2];
int rpts0s_num, rpts1s_num;
// ���ұ��߾ֲ��Ƕȱ仯��
float rpts0a[POINTS_MAX_LEN];
float rpts1a[POINTS_MAX_LEN];
int rpts0a_num, rpts1a_num;
// ���ұ��߾ֲ��Ƕȱ仯��+�Ǽ�������
float rpts0an[POINTS_MAX_LEN];
float rpts1an[POINTS_MAX_LEN];
int rpts0an_num, rpts1an_num;
// ��/������
float rptsc0[POINTS_MAX_LEN][2];
float rptsc1[POINTS_MAX_LEN][2];
int rptsc0_num, rptsc1_num;

// ����
float(*rpts)[2];
int rpts_num;
// ��һ������
float rptsn[POINTS_MAX_LEN][2];
int rptsn_num;

// Y�ǵ�
int Ypt0_rpts0s_id, Ypt1_rpts1s_id;
bool Ypt0_found, Ypt1_found;

// L�ǵ�
int Lpt0_rpts0s_id, Lpt1_rpts1s_id;
bool Lpt0_found, Lpt1_found;

// ��ֱ��
bool is_straight0, is_straight1;

//// ���
//bool is_turn0, is_turn1;

// ��ǰѲ��ģʽ
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
void process_image() {
    // ԭͼ�����ұ���
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

    // ȥ����+͸�ӱ任��mapx��mapy����������ӳ�����飩
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

    // �����˲�
    blur_points(rpts0, rpts0_num, rpts0b, (int)round(line_blur_kernel));
    rpts0b_num = rpts0_num;
    blur_points(rpts1, rpts1_num, rpts1b, (int)round(line_blur_kernel));
    rpts1b_num = rpts1_num;

    // ���ߵȾ����
    rpts0s_num = sizeof(rpts0s) / sizeof(rpts0s[0]);
    resample_points(rpts0b, rpts0b_num, rpts0s, &rpts0s_num, 1.5 * sample_dist * pixel_per_meter);
    rpts1s_num = sizeof(rpts1s) / sizeof(rpts1s[0]);
    resample_points(rpts1b, rpts1b_num, rpts1s, &rpts1s_num, 1.5 * sample_dist * pixel_per_meter);

    // ���߾ֲ��Ƕȱ仯��
    local_angle_points(rpts0s, rpts0s_num, rpts0a, (int)round(angle_dist / sample_dist));
    rpts0a_num = rpts0s_num;
    local_angle_points(rpts1s, rpts1s_num, rpts1a, (int)round(angle_dist / sample_dist));
    rpts1a_num = rpts1s_num;

    // �Ƕȱ仯�ʷǼ�������
    nms_angle(rpts0a, rpts0a_num, rpts0an, (int)round(angle_dist / sample_dist) * 2 + 1);
    rpts0an_num = rpts0a_num;
    nms_angle(rpts1a, rpts1a_num, rpts1an, (int)round(angle_dist / sample_dist) * 2 + 1);
    rpts1an_num = rpts1a_num;

    // �������߸���
    track_leftline(rpts0s, rpts0s_num, rptsc0, (int)round(angle_dist / sample_dist), pixel_per_meter * ROAD_WIDTH / 2);
    rptsc0_num = rpts0s_num;
    track_rightline(rpts1s, rpts1s_num, rptsc1, (int)round(angle_dist / sample_dist), pixel_per_meter * ROAD_WIDTH / 2);
    rptsc1_num = rpts1s_num;
}


void find_corners() {
    // ʶ��Y,L�յ�
    Ypt0_found = Ypt1_found = Lpt0_found = Lpt1_found = false;
    is_straight0 = rpts0s_num > 1. / sample_dist;//1Ϊ�����൱��  �Ⱦ������ĵ�����������
    is_straight1 = rpts1s_num > 1. / sample_dist;
    for (int i = 0; i < rpts0s_num; i++) {//����ߵĴ���
        if (rpts0an[i] == 0) continue;//�Ǽ���ֵ����
        int im1 = clip(i - (int)round(angle_dist / sample_dist), 0, rpts0s_num - 1);
        int ip1 = clip(i + (int)round(angle_dist / sample_dist), 0, rpts0s_num - 1);
        float conf = fabs(rpts0a[i]) - (fabs(rpts0a[im1]) + fabs(rpts0a[ip1])) / 2;//������ʵ�ĽǶ�
        if (conf * 180 / PI > 50 && conf * 180 / PI < 140) {
            std::cout << "��" << conf * 180 / PI << std::endl;
        }
        
        //Y�ǵ���ֵ
        if (Ypt0_found == false && 30. / 180. * PI < conf && conf < 65. / 180. * PI && i < 1.2 / sample_dist) {
            Ypt0_rpts0s_id = i;
            Ypt0_found = true;
        }
        //L�ǵ���ֵ
        if (Lpt0_found == false && 70. / 180. * PI < conf && conf < 140. / 180. * PI && i < 1.2 / sample_dist) {//i < 0.8 / sample_dist �൱���õ��ڽ���
            Lpt0_rpts0s_id = i;
            Lpt0_found = true;
        }
        //��ֱ����ֵ
        if (conf > 5. / 180. * PI && i < 1.5 / sample_dist) is_straight0 = false;//�нǶȴ���5�ȵĽǶ�ʱ���ǳ�ֱ��
        if (Ypt0_found == true && Lpt0_found == true && is_straight0 == false) break;
    }
    for (int i = 0; i < rpts1s_num; i++) {
        if (rpts1an[i] == 0) continue;
        int im1 = clip(i - (int)round(angle_dist / sample_dist), 0, rpts1s_num - 1);
        int ip1 = clip(i + (int)round(angle_dist / sample_dist), 0, rpts1s_num - 1);
        float conf = fabs(rpts1a[i]) - (fabs(rpts1a[im1]) + fabs(rpts1a[ip1])) / 2;
        if (conf * 180 / PI > 50 && conf * 180 / PI < 140) {
            std::cout << "�ң�" << conf * 180 / PI << std::endl;
        }
        if (Ypt1_found == false && 30. / 180. * PI < conf && conf < 65. / 180. * PI && i < 1.2 / sample_dist) {
            Ypt1_rpts1s_id = i;
            Ypt1_found = true;
        }
        if (Lpt1_found == false && 70. / 180. * PI < conf && conf < 140. / 180. * PI && i < 1.2 / sample_dist) {
            Lpt1_rpts1s_id = i;
            Lpt1_found = true;
        }

        if (conf > 5. / 180. * PI && i < 1.5 / sample_dist) is_straight1 = false;

        if (Ypt1_found == true && Lpt1_found == true && is_straight1 == false) break;
    }
    // Y����μ��,�������ǵ���뼰�ǵ���ſ�����
    if (Ypt0_found && Ypt1_found) {
        float dx = rpts0s[Ypt0_rpts0s_id][0] - rpts1s[Ypt1_rpts1s_id][0];
        float dy = rpts0s[Ypt0_rpts0s_id][1] - rpts1s[Ypt1_rpts1s_id][1];
        float dn = sqrtf(dx * dx + dy * dy);
        if (fabs(dn - 0.45 * pixel_per_meter) < 0.15 * pixel_per_meter) {
            float dwx = rpts0s[clip(Ypt0_rpts0s_id + 50, 0, rpts0s_num - 1)][0] -
                rpts1s[clip(Ypt1_rpts1s_id + 50, 0, rpts1s_num - 1)][0];
            float dwy = rpts0s[clip(Ypt0_rpts0s_id + 50, 0, rpts0s_num - 1)][1] -
                rpts1s[clip(Ypt1_rpts1s_id + 50, 0, rpts1s_num - 1)][1];
            float dwn = sqrtf(dwx * dwx + dwy * dwy);
            if (!(dwn > 0.7 * pixel_per_meter &&
                rpts0s[clip(Ypt0_rpts0s_id + 50, 0, rpts0s_num - 1)][0] < rpts0s[Ypt0_rpts0s_id][0] &&
                rpts1s[clip(Ypt1_rpts1s_id + 50, 0, rpts1s_num - 1)][0] > rpts1s[Ypt1_rpts1s_id][0])) {
                Ypt0_found = Ypt1_found = false;
            }
        }
        else {
            Ypt0_found = Ypt1_found = false;
        }
    }
    // L����μ�飬����ģʽ�����, ����L�ǵ���뼰�ǵ���ſ�����
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
}

void show_line() {
    //////////////////���� 2023��11��21��
    extern cv::Mat lineFrame;
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
    for (int i = 0; i < ipts0_num; i++) {
        int x = ipts0[i][0];
        int y = ipts0[i][1];
        int current_value = AT_IMAGE(&img_raw, x, y);//��ǰ�Ҷ�ֵx,y

        if (current_value < 140) {
            cv::circle(lineFrame, cv::Point(x, y), 1, cv::Scalar(0, 0, 0));
        }
        else {
            cv::circle(lineFrame, cv::Point(x, y), 1, cv::Scalar(255, 255, 255));
        }
    }
    //for (int i = 0; i < ipts1_num; i++) {
    //    int x = ipts1[i][0];
    //    int y = ipts1[i][1];
    //    int current_value = AT_IMAGE(&img_raw, x, y);//��ǰ�Ҷ�ֵx,y

    //    if (current_value < 140) {
    //        cv::circle(lineFrame, cv::Point(x, y), 1, cv::Scalar(0, 0, 0));
    //    }
    //    else {
    //        cv::circle(lineFrame, cv::Point(x, y), 1, cv::Scalar(255, 255, 255));
    //    }
    //}
    //imshow("lineFrame", lineFrame);

    // ������͸�Ӻ�ĵ�·��            
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
    imshow("nitoushi", nitoushi);

}
