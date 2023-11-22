#include "imgproc.h"
#include "common.h"
#include <assert.h>
#include <cmath>
#include "headfile.h"

extern "C" int clip(int x, int low, int up);

#define AT                  AT_IMAGE
#define AT_CLIP(img, x, y)  AT_IMAGE((img), clip((x), 0, (img)->width-1), clip((y), 0, (img)->height-1));
/* ǰ�������壺
 *   0
 * 3   1
 *   2
 */
const int dir_front[4][2] = {   {0 , -1},
                                {1 ,  0},
                                {0 ,  1},
                                {-1,  0} };
const int dir_frontleft[4][2] = {   {-1, -1},
                                    {1,  -1},
                                    {1,  1},
                                    {-1, 1} };
const int dir_frontright[4][2] = {  {1,  -1},
                                    {1,  1},
                                    {-1, 1},
                                    {-1, -1} };

// �����Թ�Ѳ��
void findline_lefthand_adaptive(image_t* img, int block_size, int clip_value, int x, int y, int pts[][2], int* num) {
    assert(img && img->data);
    assert(num && *num >= 0);
    assert(block_size > 1 && block_size % 2 == 1);
    int half = block_size / 2;//7x7��������
    int step = 0, dir = 0, turn = 0;//�߹�·������step 
    while (step < *num && half < x && x < img->width - half - 1 && half < y && y < img->height - half - 1 && turn < 4) {
        int local_thres = 140;
        //for (int dy = -half; dy <= half; dy++) {
        //    for (int dx = -half; dx <= half; dx++) {
        //        local_thres += AT(img, x + dx, y + dy);
        //    }
        //}
        //local_thres /= block_size * block_size;
        //local_thres -= clip_value;//����Ӧ��ֵ��
        int current_value = AT(img, x, y);//��ǰ�Ҷ�ֵx,y
        int front_value = AT(img, x + dir_front[dir][0], y + dir_front[dir][1]);//�Ϸ��ĻҶ�ֵ
        int frontleft_value = AT(img, x + dir_frontleft[dir][0], y + dir_frontleft[dir][1]);//��ǰ���ĻҶ�ֵ

        if (front_value < local_thres) {//���ж�ǰ��Ϊǽ,ת��90��
            dir = (dir + 1) % 4;
            turn++;
        }
        else if (frontleft_value < local_thres) {//ǰ����ǽ��ǰ��ǽΪ��ɫǰ��
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
        else {//ǰ����ǰ��������ǽǰ������ת
            x += dir_frontleft[dir][0];
            y += dir_frontleft[dir][1];
            dir = (dir + 3) % 4;
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
    }
    *num = step;
}

// �����Թ�Ѳ��
void findline_righthand_adaptive(image_t* img, int block_size, int clip_value, int x, int y, int pts[][2], int* num) {
    assert(img && img->data);
    assert(num && *num >= 0);
    assert(block_size > 1 && block_size % 2 == 1);
    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;
    
    while (step < *num && 0 < x && x < img->width - 1 && 0 < y && y < img->height - 1 && turn < 4) {
        int local_thres = 140;
        //for (int dy = -half; dy <= half; dy++) {
        //    for (int dx = -half; dx <= half; dx++) {
        //        local_thres += AT(img, x + dx, y + dy);
        //    }
        //}
        //local_thres /= block_size * block_size;
        //local_thres -= clip_value;

        int current_value = AT(img, x, y);
        int front_value = AT(img, x + dir_front[dir][0], y + dir_front[dir][1]);
        int frontright_value = AT(img, x + dir_frontright[dir][0], y + dir_frontright[dir][1]);

        if (front_value < local_thres) {
            dir = (dir + 3) % 4;
            turn++;
        }
        else if (frontright_value < local_thres) {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
        else {
            x += dir_frontright[dir][0];
            y += dir_frontright[dir][1];
            dir = (dir + 1) % 4;
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
    }
    *num = step;
}

// �㼯�����˲�
void blur_points(float pts_in[][2], int num, float pts_out[][2], int kernel) {
    assert(kernel % 2 == 1);
    int half = kernel / 2;
    for (int i = 0; i < num; i++) {
        pts_out[i][0] = pts_out[i][1] = 0;
        for (int j = -half; j <= half; j++) {
            pts_out[i][0] += pts_in[clip(i + j, 0, num - 1)][0] * (half + 1 - abs(j));
            pts_out[i][1] += pts_in[clip(i + j, 0, num - 1)][1] * (half + 1 - abs(j));
        }
        pts_out[i][0] /= (2 * half + 2) * (half + 1) / 2;
        pts_out[i][1] /= (2 * half + 2) * (half + 1) / 2;//����˲��������
    }
}

// �㼯�Ⱦ����  ʹ�߹��Ĳ���ǰ���߶εľ���Ϊ`dist`
void resample_points(float pts_in[][2], int num1, float pts_out[][2], int* num2, float dist) {
    int remain = 0, len = 0;
    for (int i = 0; i < num1 - 1 && len < *num2; i++) {
        float x0 = pts_in[i][0];
        float y0 = pts_in[i][1];
        float dx = pts_in[i + 1][0] - x0;
        float dy = pts_in[i + 1][1] - y0;
        float dn = sqrt(dx * dx + dy * dy);
        dx /= dn;
        dy /= dn;

        while (remain < dn && len < *num2) {
            x0 += dx * remain;//���Բ�ֵ����ɨ���������x�������ϳ�һ����ֵ
            pts_out[len][0] = x0;
            y0 += dy * remain;//ͬ��
            pts_out[len][1] = y0;

            len++;
            dn -= remain;//��ȥ�趨����涨���궨��
            remain = dist;
        }
        remain -= dn;
    }
    *num2 = len;//���ر��߳���
}

// �㼯�ֲ��Ƕȱ仯��  ����Ϊdist
void local_angle_points(float pts_in[][2], int num, float angle_out[], int dist) {
    for (int i = 0; i < num; i++) {
        if (i <= 0 || i >= num - 1) {
            angle_out[i] = 0;
            continue;
        }
        float dx1 = pts_in[i][0] - pts_in[clip(i - dist, 0, num - 1)][0];//��ǰ����ʮ������  //clip�൱�ڴ���һ��x�����޷�
        float dy1 = pts_in[i][1] - pts_in[clip(i - dist, 0, num - 1)][1];
        float dn1 = sqrtf(dx1 * dx1 + dy1 * dy1);
        float dx2 = pts_in[clip(i + dist, 0, num - 1)][0] - pts_in[i][0];//��ʮ������
        float dy2 = pts_in[clip(i + dist, 0, num - 1)][1] - pts_in[i][1];
        float dn2 = sqrtf(dx2 * dx2 + dy2 * dy2);
        float c1 = dx1 / dn1;//cos
        float s1 = dy1 / dn1;//sin
        float c2 = dx2 / dn2;
        float s2 = dy2 / dn2;
        angle_out[i] = atan2f(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);//arctan����Ƕ�
    }
}

// �Ƕȱ仯�ʷǼ�������
void nms_angle(float angle_in[], int num, float angle_out[], int kernel) {
    assert(kernel % 2 == 1);
    int half = kernel / 2;
    for (int i = 0; i < num; i++) {
        angle_out[i] = angle_in[i];
        for (int j = -half; j <= half; j++) {
            if (fabs(angle_in[clip(i + j, 0, num - 1)]) > fabs(angle_out[i])) {
                angle_out[i] = 0;
                break;
            }
        }
    }
}

// ����߸�������
void track_leftline(float pts_in[][2], int num, float pts_out[][2], int approx_num, float dist) {
    for (int i = 0; i < num; i++) {
        float dx = pts_in[clip(i + approx_num, 0, num - 1)][0] - pts_in[clip(i - approx_num, 0, num - 1)][0];
        float dy = pts_in[clip(i + approx_num, 0, num - 1)][1] - pts_in[clip(i - approx_num, 0, num - 1)][1];
        float dn = sqrt(dx * dx + dy * dy);
        dx /= dn;//sin
        dy /= dn;//cos
        pts_out[i][0] = pts_in[i][0] - dy * dist;
        pts_out[i][1] = pts_in[i][1] + dx * dist;//ͨ���������sin  cos����һ��ԭ��������λ�ü��������ߵķ����������ӳ��������
    }
}

// �ұ��߸�������
void track_rightline(float pts_in[][2], int num, float pts_out[][2], int approx_num, float dist) {
    for (int i = 0; i < num; i++) {
        float dx = pts_in[clip(i + approx_num, 0, num - 1)][0] - pts_in[clip(i - approx_num, 0, num - 1)][0];
        float dy = pts_in[clip(i + approx_num, 0, num - 1)][1] - pts_in[clip(i - approx_num, 0, num - 1)][1];
        float dn = sqrt(dx * dx + dy * dy);
        dx /= dn;
        dy /= dn;
        pts_out[i][0] = pts_in[i][0] + dy * dist;
        pts_out[i][1] = pts_in[i][1] - dx * dist;
    }
}

