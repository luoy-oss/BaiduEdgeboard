#define CAR_DEBUGE
#define CIRCLE_DEBUGE
//#define CROSS_DEBUGE
#define GARAGE_DEBUGE
#include "main.h"
#include "../code/headfile.h"

#include "../code/common.h"

#include "../code/debugger.h"
#include "../code/imgproc.h"
#include "../code/camera_param.h"

#include "../recognition/track_recognition.h"
#include "../recognition/cross_recognition.h"
#include "../recognition/circle_recognition.h"
#include "../recognition/garage_recognition.h"
using namespace cv;

//�û�����ͼ������ֱ�ӷ������ָ������Ϳ���
//���ʷ�ʽ�ǳ��򵥣�����ֱ��ʹ���±�ķ�ʽ����
//������ʵ�10�� 50�еĵ㣬mt9v03x_csi_image[10][50]�Ϳ�����
//uint8(*mt9v03x_csi_image)[MT9V03X_CSI_W];
uint8_t img_thres_data[ROWSIMAGE][COLSIMAGE];
uint8_t img_line_data[ROWSIMAGE][COLSIMAGE];

//image_t img_raw(NULL, COLSIMAGE, ROWSIMAGE);
image_t img_thres((uint8_t*)img_thres_data, COLSIMAGE, ROWSIMAGE);
image_t img_line((uint8_t*)img_line_data, COLSIMAGE, ROWSIMAGE);

// line_show_sample��cross.c�е�draw_cross()�б�����
bool line_show_sample = true;

//Ԥ�����
float thres = 140;
float block_size = 7;
float clip_value = 2;

float begin_x = 15;
float begin_y = 167; 

float line_blur_kernel = 7;
float pixel_per_meter = 102;//ƽ�����أ��������
float sample_dist = 0.02;
float angle_dist = 0.2;
float far_rate = 0.5;
float aim_distance = 0.68;
bool adc_cross = false;

Mat lineFrame;
Mat nitoushi;
int main() {

	Mat frame;
	VideoCapture cap(0);
	//VideoCapture cap("lx.mp4");
	cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
	//
	//
	//frame = imread("f4.jpg");
	////		frame = imread("f (2).jpg");
	//cvtColor(frame, frame, COLOR_BGR2GRAY);
	//threshold(frame, frame, 0, 255, THRESH_OTSU);
	//frameTOimg_raw(frame);
	//process_image();
	//imshow("frame", frame);
	//waitKey(1000000);
	while (1) {
		//circle_type = CIRCLE_LEFT_IN;
		//circle_type = CIRCLE_RIGHT_IN;
				lineFrame = Mat::zeros(cv::Size(320, 240), CV_8UC1);
		nitoushi = Mat::zeros(cv::Size(320, 240), CV_8UC1);
		cap >> frame;
		//frame = imread("f4.jpg");
		cvtColor(frame, frame, COLOR_BGR2GRAY);
		threshold(frame, frame, 0, 255, THRESH_OTSU);
		frameTOimg_raw(frame);
		process_image();
		/*{
			show_line();
		}*/
		find_corners();
		// Ԥ�����,��̬Ч������
		aim_distance = 0.58;
		// �������٣��л�Ѳ�߷���  ������Բ
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
		
		// ��������߼��(���������ȼ��ߣ����ȼ��)
		//check_garage();

		// �ֱ���ʮ�� ���� ��Բ��, ʮ�����ȼ����
		if (true/*garage_type == GARAGE_NONE */)
			check_cross();
		if (/*garage_type == GARAGE_NONE && */cross_type == CROSS_NONE)
			check_circle();
		if (cross_type != CROSS_NONE) {
			circle_type = CIRCLE_NONE;
		}
		//���ݼ����ִ��ģʽ
		if (cross_type != CROSS_NONE) {
		//std::cout << "run cross: "<< std::endl;
		//std::cout << "cross_type:"<< cross_type << std::endl;

			run_cross();
		}
		if (cross_type != CROSS_NONE) run_cross();
		if (circle_type != CIRCLE_NONE) run_circle();
		//if (garage_type != GARAGE_NONE) run_garage();
		
		// ���߸���
		if (cross_type != CROSS_IN) {
			if (track_type == TRACK_LEFT) {
				rpts = rptsc0;
				rpts_num = rptsc0_num;
			}
			else {
				rpts = rptsc1;
				rpts_num = rptsc1_num;
			}
		}
		else {
			//ʮ�ָ���Զ�߿���
			if (track_type == TRACK_LEFT) {
				track_leftline(far_rpts0s + far_Lpt0_rpts0s_id, far_rpts0s_num - far_Lpt0_rpts0s_id, rpts,
					(int)round(angle_dist / sample_dist), pixel_per_meter * ROAD_WIDTH / 2);
				rpts_num = far_rpts0s_num - far_Lpt0_rpts0s_id;
			}
			else {
				track_rightline(far_rpts1s + far_Lpt1_rpts1s_id, far_rpts1s_num - far_Lpt1_rpts1s_id, rpts,
					(int)round(angle_dist / sample_dist), pixel_per_meter * ROAD_WIDTH / 2);
				rpts_num = far_rpts1s_num - far_Lpt1_rpts1s_id;
			}
		}
		clear_image(&img_line);
		draw_circle();
		draw_cross();
#ifdef CAR_DEBUGE
	#ifdef CIRCLE_DEBUGE
		COUT1(circle_type);
		if (circle_type == CIRCLE_NONE) {
			COUT1("CIRCLE_NONE");
		}
	#endif
	#ifdef CROSS_DEBUGE
		if (cross_type == CROSS_NONE) {
			COUT1("CROSS_NONE");
		}
	#endif
#endif
		// ���Ƶ�·��            
		for (int i = 0; i < rpts0s_num; i++) {
				AT_IMAGE(&img_line, clip(rpts0s[i][0], 0, img_line.width - 1),
					clip(rpts0s[i][1], 0, img_line.height - 1)) = 255;
		}
		for (int i = 0; i < rpts1s_num; i++) {
			AT_IMAGE(&img_line, clip(rpts1s[i][0], 0, img_line.width - 1),
				clip(rpts1s[i][1], 0, img_line.height - 1)) = 255;
		}
		for (int i = 0; i < rptsn_num; i++) {
			AT_IMAGE(&img_line, clip(rptsn[i][0], 0, img_line.width - 1),
				clip(rptsn[i][1], 0, img_line.height - 1)) = 255;
		}
		// ����ê��
		int aim_idx = clip(round(aim_distance / sample_dist), 0, rptsn_num - 1);
		draw_x(&img_line, rptsn[aim_idx][0], rptsn[aim_idx][1], 3, 255);
		// ���ƽǵ�
		if (Lpt0_found) {
			draw_x(&img_line, rpts0s[Lpt0_rpts0s_id][0], rpts0s[Lpt0_rpts0s_id][1], 3, 255);
		}
		if (Lpt1_found) {
			draw_x(&img_line, rpts1s[Lpt1_rpts1s_id][0], rpts1s[Lpt1_rpts1s_id][1], 3, 255);
		}

		show_line();
		imshow("frame", frame);
		waitKey(1);
	}
	return 0;
}