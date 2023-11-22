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

//用户访问图像数据直接访问这个指针变量就可以
//访问方式非常简单，可以直接使用下标的方式访问
//例如访问第10行 50列的点，mt9v03x_csi_image[10][50]就可以了
//uint8(*mt9v03x_csi_image)[MT9V03X_CSI_W];
uint8_t img_thres_data[ROWSIMAGE][COLSIMAGE];
uint8_t img_line_data[ROWSIMAGE][COLSIMAGE];

//image_t img_raw(NULL, COLSIMAGE, ROWSIMAGE);
image_t img_thres((uint8_t*)img_thres_data, COLSIMAGE, ROWSIMAGE);
image_t img_line((uint8_t*)img_line_data, COLSIMAGE, ROWSIMAGE);

// line_show_sample在cross.c中的draw_cross()中被调用
bool line_show_sample = true;

//预瞄距离
float thres = 140;
float block_size = 7;
float clip_value = 2;

float begin_x = 15;
float begin_y = 167; 

float line_blur_kernel = 7;
float pixel_per_meter = 102;//平移像素，拟合中线
float sample_dist = 0.02;
float angle_dist = 0.2;
float far_rate = 0.5;
float aim_distance = 0.68;
bool adc_cross = false;

Mat lineFrame;
Mat nitoushi;
int main() {

	Mat frame;
	//VideoCapture cap(0);
	VideoCapture cap("lx.mp4");
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
		// 预瞄距离,动态效果更佳
		aim_distance = 0.58;
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
		
		// 车库斑马线检查(斑马线优先级高，最先检查)
		//check_garage();

		// 分别检查十字 三叉 和圆环, 十字优先级最高
		if (true/*garage_type == GARAGE_NONE */)
			check_cross();
		if (/*garage_type == GARAGE_NONE && */cross_type == CROSS_NONE)
			check_circle();
		if (cross_type != CROSS_NONE) {
			circle_type = CIRCLE_NONE;
		}
		//根据检查结果执行模式
		if (cross_type != CROSS_NONE) {
		//std::cout << "run cross: "<< std::endl;
		//std::cout << "cross_type:"<< cross_type << std::endl;

			run_cross();
		}
		if (cross_type != CROSS_NONE) run_cross();
		if (circle_type != CIRCLE_NONE) run_circle();
		//if (garage_type != GARAGE_NONE) run_garage();
		
		// 中线跟踪
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
			//十字根据远线控制
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

		{
			// 车轮对应点(纯跟踪起始点)
			float cx = mapx[COLSIMAGE / 2][(int)(ROWSIMAGE * 0.78f)];
			float cy = mapy[COLSIMAGE / 2][(int)(ROWSIMAGE * 0.78f)];
			
			// 找最近点(起始点中线归一化)
			float min_dist = 1e10;
			int begin_id = -1;
			for (int i = 0; i < rpts_num; i++) {
				float dx = rpts[i][0] - cx;
				float dy = rpts[i][1] - cy;
				float dist = sqrt(dx * dx + dy * dy);
				if (dist < min_dist) {
					min_dist = dist;
					begin_id = i;
				}
			}
		
			// 特殊模式下，不找最近点(由于边线会绕一圈回来，导致最近点为边线最后一个点，从而中线无法正常生成)
			if (/*garage_type == GARAGE_IN_LEFT || garage_type == GARAGE_IN_RIGHT || */cross_type == CROSS_IN) begin_id = 0;
			
			// 中线有点，同时最近点不是最后几个点
			if (begin_id >= 0 && rpts_num - begin_id >= 3) {
				// 归一化中线
				rpts[begin_id][0] = cx;
				rpts[begin_id][1] = cy;
				rptsn_num = sizeof(rptsn) / sizeof(rptsn[0]);
				resample_points(rpts + begin_id, rpts_num - begin_id, rptsn, &rptsn_num, sample_dist * pixel_per_meter);

				// 远预锚点位置
				int aim_idx = clip(round(aim_distance / sample_dist), 0, rptsn_num - 1);
				// 近预锚点位置
				int aim_idx_near = clip(round(0.25 / sample_dist), 0, rptsn_num - 1);

				// 计算远锚点偏差值
				float dx = rptsn[aim_idx][0] - cx;
				float dy = cy - rptsn[aim_idx][1] + 0.2 * pixel_per_meter;
				float dn = sqrt(dx * dx + dy * dy);
				float error = -atan2f(dx, dy) * 180 / PI;
				assert(!isnan(error));

				// 若考虑近点远点,可近似构造Stanley算法,避免撞路肩
				// 计算近锚点偏差值
				float dx_near = rptsn[aim_idx_near][0] - cx;
				float dy_near = cy - rptsn[aim_idx_near][1] + 0.2 * pixel_per_meter;
				float dn_near = sqrt(dx_near * dx_near + dy_near * dy_near);
				float error_near = -atan2f(dx_near, dy_near) * 180 / PI;
				assert(!isnan(error_near));

				// 远近锚点综合考虑
				//angle = pid_solve(&servo_pid, error * far_rate + error_near * (1 - far_rate));
				// 根据偏差进行PD计算
				//float angle = pid_solve(&servo_pid, error);

				//// 纯跟踪算法(只考虑远点)
				//float pure_angle = -atanf(pixel_per_meter * 2 * 0.2 * dx / dn / dn) / PI * 180 / SMOTOR_RATE;
				//angle = pid_solve(&servo_pid, pure_angle);
				//angle = MINMAX(angle, -14.5, 14.5);

				////非上坡电感控制,PD计算之后的值用于寻迹舵机的控制
				//if (enable_adc) {
				//	// 当前上坡模式，不控制舵机，同时清除所有标志
				//	circle_type = CIRCLE_NONE;
				//	cross_type = CROSS_NONE;
				//	//garage_type = GARAGE_NONE;
				//}
				//else if (adc_cross && cross_type == CROSS_IN) {
				//	// 当前是十字模式，同时启用了电感过十字，则不控制舵机
				//}
				//else {
				//	//smotor1_control(servo_duty(SMOTOR1_CENTER + angle));
				//}

			}
			else {
				// 中线点过少(出现问题)，则不控制舵机
				rptsn_num = 0;
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
		// 绘制道路线            
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
		// 绘制锚点
		int aim_idx = clip(round(aim_distance / sample_dist), 0, rptsn_num - 1);
		draw_x(&img_line, rptsn[aim_idx][0], rptsn[aim_idx][1], 3, 255);
		// 绘制角点
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