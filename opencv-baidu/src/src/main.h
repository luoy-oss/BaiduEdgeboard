#ifndef __MAIN_H__
#define __MAIN_H__

#define CAR_DEBUG
#define CAR_SAVEIMG
//#define CIRCLE_DEBUG
//#define CROSS_DEBUG
//#define GARAGE_DEBUG

#include "../code/headfile.h"
#include "../code/imgproc.h"
#include "../code/utils.h"
#include "../code/common.h"

#define COUT1(x) (std::cout<<(x)<<std::endl)
#define COUT2(x,y) (std::cout<<(x)<<","<<(y)<<std::endl)

extern float thres;                 // ��ֵ����ֵ����Ҫ��������ʼ��(����ʹ������Ӧ��ֵ����ʹ�ø���ֵ)
extern float block_size;            // ����Ӧ��ֵ��block��С
extern float clip_value;            // ����Ӧ��ֵ����ֵ�ü���
extern float begin_x;               // ��ʼ�����ͼ�����ĵ�����ƫ����
extern float begin_y;               // ��ʼ�����ͼ��ײ�������ƫ����
extern float line_blur_kernel;      // ���������˲��˵Ĵ�С
extern float pixel_per_meter;       // ����ͼ�У�ÿ�����ض�Ӧ�ĳ���
extern float sample_dist;           // ���ߵȾ�����ļ��
extern float angle_dist;            // �������ת��ʱ�����������ľ���
extern float aim_distance;          // Ԥê�㳤��
extern float far_rate;              //
extern bool adc_cross;              // �Ƿ����õ�й�ʮ��

extern image_t img_raw;
extern image_t img_thres;
extern image_t img_line;

extern bool line_show_sample;
extern bool line_show_blur;
extern bool track_left;

extern float angle;

#define ROAD_WIDTH      (0.45)
#define POINTS_MAX_LEN  (ROWSIMAGE)

#define FAR_POINTS_MAX_LEN  (POINTS_MAX_LEN)

// ԭͼ���ұ���
extern int ipts0[POINTS_MAX_LEN][2];
extern int ipts1[POINTS_MAX_LEN][2];
extern int ipts0_num, ipts1_num;
// �任�����ұ���
extern float rpts0[POINTS_MAX_LEN][2];
extern float rpts1[POINTS_MAX_LEN][2];
extern int rpts0_num, rpts1_num;
// �任�����ұ���+�˲�
extern float rpts0b[POINTS_MAX_LEN][2];
extern float rpts1b[POINTS_MAX_LEN][2];
extern int rpts0b_num, rpts1b_num;
// �任�����ұ���+�Ⱦ����
extern float rpts0s[POINTS_MAX_LEN][2];
extern float rpts1s[POINTS_MAX_LEN][2];
extern int rpts0s_num, rpts1s_num;
// ���ұ��߾ֲ��Ƕȱ仯��
extern float rpts0a[POINTS_MAX_LEN];
extern float rpts1a[POINTS_MAX_LEN];
extern int rpts0a_num, rpts1a_num;
// ���ұ��߾ֲ��Ƕȱ仯��+�Ǽ�������
extern float rpts0an[POINTS_MAX_LEN];
extern float rpts1an[POINTS_MAX_LEN];
extern int rpts0an_num, rpts1an_num;
// ��/������
extern float rptsc0[POINTS_MAX_LEN][2];
extern float rptsc1[POINTS_MAX_LEN][2];
extern int rptsc0_num, rptsc1_num;
// ����
extern float(*rpts)[2];
extern int rpts_num;
// ��һ������
extern float rptsn[POINTS_MAX_LEN][2];
extern int rptsn_num;

// Y�ǵ�
extern int Ypt0_rpts0s_id, Ypt1_rpts1s_id;
extern bool Ypt0_found, Ypt1_found;

// L�ǵ�
extern int Lpt0_rpts0s_id, Lpt1_rpts1s_id;
extern bool Lpt0_found, Lpt1_found;

//����Ϊʮ�ֿ���ѰԶ�߲���,�㷨�볣��Ѱ����ͬ

extern bool far_Lpt0_found, far_Lpt1_found;
extern int far_Lpt0_rpts0s_id, far_Lpt1_rpts1s_id;
//ԭͼԶ�����ұ���
extern int far_ipts0[FAR_POINTS_MAX_LEN][2];
extern int far_ipts1[FAR_POINTS_MAX_LEN][2];
extern int far_ipts0_num, far_ipts1_num;
//�任������Զ����
extern float far_rpts0[FAR_POINTS_MAX_LEN][2];
extern float far_rpts1[FAR_POINTS_MAX_LEN][2];
extern int far_rpts0_num, far_rpts1_num;
//�任������Զ����+�˲�
extern float far_rpts0b[FAR_POINTS_MAX_LEN][2];
extern float far_rpts1b[FAR_POINTS_MAX_LEN][2];
extern int far_rpts0b_num, far_rpts1b_num;
//�任������Զ����+�Ⱦ����
extern float far_rpts0s[FAR_POINTS_MAX_LEN][2];
extern float far_rpts1s[FAR_POINTS_MAX_LEN][2];
extern int far_rpts0s_num, far_rpts1s_num;
// ����Զ���߾ֲ��Ƕȱ仯��
extern float far_rpts0a[FAR_POINTS_MAX_LEN];
extern float far_rpts1a[FAR_POINTS_MAX_LEN];
extern int far_rpts0a_num, far_rpts1a_num;
// ����Զ���߾ֲ��Ƕȱ仯��+�Ǽ�������
extern float far_rpts0an[FAR_POINTS_MAX_LEN];
extern float far_rpts1an[FAR_POINTS_MAX_LEN];
extern int far_rpts0an_num, far_rpts1an_num;

extern bool is_straight0, is_straight1;

enum track_type_e {
    TRACK_LEFT,
    TRACK_RIGHT,
};
extern enum track_type_e track_type;


extern double stdevLeft;    // ��Ե����
extern double stdevRight;   // ��Ե����
extern float radius;       // Բ���뾶


// extern long double bias_p;
// extern long double bias_p_last;
extern long double bias_i;
// extern long double bias_d;
#endif //! __MAIN_H__