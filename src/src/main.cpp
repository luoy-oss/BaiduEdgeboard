
#include "main.h"
#include "../code/headfile.h"

#include "../code/common.h"
#include "../code/smotor.h"
#include "../code/pid.h"

#include "../code/debugger.h"
#include "../code/imgproc.h"
#include "../code/camera_param.h"
#include "../code/motor.h"
#include "../code/detection.hpp"

#include "../recognition/track_recognition.h"
#include "../recognition/cross_recognition.h"
#include "../recognition/circle_recognition.h"
#include "../recognition/garage_recognition.h"

#include "../code/value_config.cpp"

#include "../detection/danger.cpp"
#include "../detection/rescue.cpp"
#include "../detection/racing.cpp"
#include "../detection/bridge.cpp"

#include "../code/controlCenter.h"

#include "../code/uart.h"

#include "V4L2Capture.cpp"

using namespace cv;

bool thread_end = false;

Mat frame;			//RGB图像
Mat ai_frame;		//RGB图像
Mat rgb_frame;		//RGB图像
Mat danger_frame;
Mat lineFrame;		//原始赛道寻线
Mat nitoushi;		//逆透视赛道线
Mat imageCorrect;	//赛道元素


//用户访问图像数据直接访问这个指针变量就可以
//访问方式非常简单，可以直接使用下标的方式访问
//例如访问第10行 50列的点，mt9v03x_csi_image[10][50]就可以了
//uint8(*mt9v03x_csi_image)[MT9V03X_CSI_W];
uint8_t img_thres_data[ROWSIMAGE][COLSIMAGE];
uint8_t img_line_data[ROWSIMAGE][COLSIMAGE];

image_t img_thres((uint8_t*)img_thres_data, COLSIMAGE, ROWSIMAGE);
image_t img_line((uint8_t*)img_line_data, COLSIMAGE, ROWSIMAGE);

// line_show_sample在cross.c中的draw_cross()中被调用
bool line_show_sample = true;

//预瞄距离
float thres = 140;
float block_size = 7;
float clip_value = 2;

float begin_x = 15;
const float BEGIN_Y = 190;
const int X1_BOUNDARY = 124;    // x1 < 115根据填充白色多边形循迹边界判断
const int X2_BOUNDARY = 200;	// x2 > 213根据填充白色多边形循迹边界判断
float begin_y = BEGIN_Y;

float line_blur_kernel = 7;	//原7
float pixel_per_meter = 95;	//平移像素，拟合中线
float sample_dist = 0.02;	//原0.02
float angle_dist = 0.2;		//原0.2
float aim_distance = 0.68;	//原7

std::shared_ptr<Driver> driver = nullptr; // 初始化串口驱动
ControlCenter controlCenter;
ValueConfig valueConfig;

// long double lastmidAdd = 0.0;

float speed_set = 0;
bool is_start = true;
bool stop = true;
int start_speed_count = 0;

int buzzer = 0;

double FPS = 0;

double line_time = 0;
double ai_time = 0;

int slow_count = 0;
int fast_count = 0;
long double data_bias_p = 0;
long double data_bias_p_last = 0;
long double data_bias_d = 0;

void debug_show();

pthread_mutex_t mutex_cap;
pthread_t cap_thread;

pthread_mutex_t mutex_ai;
pthread_t ai_thread;

pthread_mutex_t mutex_line;
pthread_t line_thread;

std::atomic<bool> ai_init = false;
std::atomic<bool> ai_run = false;
std::atomic<bool> AI_ENABLE = false;
pthread_mutex_t* ai_mutex = nullptr;

Danger danger;
Rescue rescue;
Racing racing;
Bridge bridge;
void* ai_detection(void* arg) {
	ai_mutex = (pthread_mutex_t*)arg;

	// 目标检测类(AI模型文件)
	shared_ptr<Detection> detection = make_shared<Detection>("../res/model/yolov3_mobilenet_v1");
	detection->score_init(	valueConfig.params.cone_score,
							valueConfig.params.bomb_score,
							valueConfig.params.block_score,
							valueConfig.params.bridge_score,
							valueConfig.params.crosswalk_score,
							valueConfig.params.people_score,
							valueConfig.params.car_score,
							valueConfig.params.car_label_score);

	cpu_set_t mask;	// cpu核的集合
    // cpu_set_t get;	// 获取在集合中的cpu
    
	CPU_ZERO(&mask);	// 将集合置为空集
    CPU_SET(4, &mask);	// 设置亲和力值 cpu 4
    CPU_SET(5, &mask);	// 设置亲和力值 cpu 5
    CPU_SET(6, &mask);	// 设置亲和力值 cpu 6
    CPU_SET(7, &mask);	// 设置亲和力值 cpu 7
    
	// 设置线程cpu亲和力
    if(sched_setaffinity(0,sizeof(cpu_set_t), &mask)==-1) {
        printf("line_process warning: could not set CPU affinity, continuing...\n");
    }
	
	danger._init_(BLOCK_POI, BOMB_Y_TOP, CONE_Y_TOP, CONE_Y_DOWN, MID_L, MID_R, BLOCK_MID_L,BLOCK_MID_R);
	racing._init_(RACING_STOP_COUNT, AVOID_LEFT_CAR, AVOID_RIGHT_CAR, CRUSH_LEFT_CAR, CRUSH_RIGHT_CAR, STOP_LEFT_CAR, STOP_RIGHT_CAR);
	
	ai_init = true;
	while(1) {
		if(ai_init && !ai_frame.empty()) {
			pthread_mutex_lock(ai_mutex);
			ai_run = true;
			pthread_mutex_unlock(ai_mutex);

			// 处理帧时长监测
			static auto ai_preTime = std::chrono::duration_cast<std::chrono::milliseconds>(
									std::chrono::system_clock::now().time_since_epoch())
									.count();
			auto ai_startTime = std::chrono::duration_cast<std::chrono::milliseconds>(
								std::chrono::system_clock::now().time_since_epoch())
								.count();
			std::cout << "AI time: " <<ai_startTime - ai_preTime << "ms \t line time: " << line_time << "ms" << std::endl;
			ai_preTime = ai_startTime;
			ai_time = ai_startTime - ai_preTime;

			detection->inference(ai_frame);
			
			if(it_AI /*&& circle_type <= CIRCLE_RIGHT_BEGIN && cross_type == CROSS_NONE*/) {
				if(it_DANGER)danger.process(detection->results);	
				if(it_RESCUE)rescue.process(detection->results);
				if(it_RACING)racing.process(detection->results);
				
				if(it_BRIDGE)bridge.process(detection->results);
			}

			if(DANGER_ENABLE || RESCUE_ENABLE || RACING_ENABLE) {
				pthread_mutex_lock(ai_mutex);
				AI_ENABLE = true;
				pthread_mutex_unlock(ai_mutex);
			}

			if(saveImage) {
				static int i = 1;
				std::string filename = "../frame/" + std::to_string(i++) + ".jpg";
				imwrite(filename, ai_frame);
			}

			if (nts_debug) {
				Mat nitoushi;
				nitoushi = Mat::zeros(cv::Size(320, 240), CV_8UC1);

				draw_circle();
				draw_cross();
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
				for (int i = 0; i < circle_rptss_num; i++) {
					AT_IMAGE(&img_line, clip(circle_rptss[i][0], 0, img_line.width - 1),
						clip(circle_rptss[i][1], 0, img_line.height - 1)) = 255;
				}
				

				// 绘制锚点
				int aim_idx = clip(round(aim_distance / sample_dist), 0, rptsn_num - 1);
				draw_x(&img_line, rptsn[aim_idx][0], rptsn[aim_idx][1], 3, 255);
				// 绘制角点
				if (Lpt0_found) {
					draw_x(&img_line, rpts0s[Lpt0_rpts0s_id][0], rpts0s[Lpt0_rpts0s_id][1], 7, 255);
				}
				if (Lpt1_found) {
					draw_x(&img_line, rpts1s[Lpt1_rpts1s_id][0], rpts1s[Lpt1_rpts1s_id][1], 7, 255);
				}
				
				if (circle_Lpt_found) {
					draw_x(&img_line, circle_rptss[circle_Lpt_rptss_id][0], circle_rptss[circle_Lpt_rptss_id][1], 5, 255);
				}	
				
				if(true) {
					vector<PredictResult> predict = detection->results;
					for(const auto& i : predict) {
						if (i.type == LABEL_CONE /*&& (i.y + i.height) > ROWSIMAGE * 0.4*/)  {
							int cone_poi_x = mapx[(int)(i.y + i.height/2)][(int)(i.x + i.width/2.0)];
							int cone_poi_y = mapy[(int)(i.y + i.height/2)][(int)(i.x + i.width/2.0)];
							draw_x(&img_line, cone_poi_x, cone_poi_y, 5, 255);

						}

						if(i.type == LABEL_BLOCK && i.y + i.height > CONE_Y_TOP) {
							int block_poi_x = mapx[(int)(i.y + i.height/2)][(int)(i.x + i.width/2.0)];
							int block_poi_y = mapy[(int)(i.y + i.height/2)][(int)(i.x + i.width/2.0)];
							draw_o(&img_line, block_poi_x, block_poi_y, 5, 255);
						}
					}
				}


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

				imshow("nitoushi_thread", nitoushi);
				waitKey(1);

				if(saveImage) {
					static int ntsi = 1;
					std::string filename = "../frame/nitoushi" + std::to_string(ntsi++) + ".jpg";
					imwrite(filename, nitoushi);
				}
				clear_image(&img_line);
			}

			if(track_debug) {
				imageCorrect = ai_frame.clone();
				debug_show();
				imshow("imageCorrect_thread", imageCorrect);
				waitKey(1);

				if(saveImage) {
					static int ic = 1;
					std::string filename = "../frame/imageCorrect" + std::to_string(ic++) + ".jpg";
					imwrite(filename, imageCorrect);
				}
			}

			if(ai_debug) {
				detection->drawBox(ai_frame);
				imshow("imageAi_thread", ai_frame);
				waitKey(1);
				if(saveImage) {
					static int ai = 1;
					std::string aifilename = "../frame/imageAi" + std::to_string(ai++) + ".jpg";
					imwrite(aifilename, ai_frame);
				}
			}

			pthread_mutex_lock(ai_mutex);
			ai_frame = cv::Mat();
			ai_run = false;
			pthread_mutex_unlock(ai_mutex);
		}

		if(thread_end) {
			std::cout << "thread() : ai_detection 退出." << std::endl;
			return NULL;
		}
	}

	return NULL;
}

std::atomic<bool> line_init = false;
std::atomic<bool> line_run = false;
pthread_mutex_t* line_mutex = nullptr;
void* trackline_process(void* arg);

// 图像获取线程绑定cpu1
pthread_mutex_t* cap_mutex = nullptr;
void* frame_get(void* arg);
int main() {
	controlCenter.valueInit(valueConfig);

	if(ai_debug) {
		cv::namedWindow("imageAi_thread", 0);
		cv::resizeWindow("imageAi_thread", 320, 240);
	}

	if(track_debug) {
		cv::namedWindow("imageCorrect_thread", 0);
		cv::resizeWindow("imageCorrect_thread", 320, 240);
	}

	if(nts_debug) {
		cv::namedWindow("nitoushi_thread", 0);
		cv::resizeWindow("nitoushi_thread", 320, 240);
	}

	{
		// USB转串口的设备名为 /dev/ttyUSB0
		driver = std::make_shared<Driver>("/dev/ttyUSB0", BaudRate::BAUD_115200);
		if (driver == nullptr) {
			std::cout << "Create Uart-Driver Error!" << std::endl;
			return -1;
		}
		// 串口初始化，打开串口设备及配置串口数据格式
		int ret = driver->open();
		if (ret != 0) {
			std::cout << "Uart Open failed!" << std::endl;
			return -1;
		}
	}

	usleep(500000);


#ifdef CAR_DEBUG
		cv::namedWindow("danger",0);
		cv::resizeWindow("danger", 320, 240);
#endif

	// 图像获取线程与cpu0 1绑定
	if(pthread_mutex_init(&mutex_cap, NULL) < 0) {
		std::cout<<"sem_init error"<<std::endl;
		exit(-1);
	}
	pthread_create(&cap_thread, NULL, frame_get, &mutex_cap);

	// 寻线线程与cpu2 3绑定
	if(pthread_mutex_init(&mutex_line, NULL) < 0) {
		std::cout<<"sem_init error"<<std::endl;
		exit(-1);
	}
	pthread_create(&line_thread, NULL, trackline_process, &mutex_line);
	usleep(1000000);
	
	// AI线程与cpu4 5 6 7绑定
	if(pthread_mutex_init(&mutex_ai, NULL) < 0) {
		std::cout<<"sem_init error"<<std::endl;
		exit(-1);
	}
	pthread_create(&ai_thread, NULL, ai_detection, &mutex_ai);

	std::cout << "AI线程创建等待..." << std::endl;
	std::cout << "AI线程创建等待..." << std::endl;
	std::cout << "AI线程创建等待..." << std::endl;
	usleep(1500000);



	std::cout << "程序启动！" << std::endl;
	while (1) {
		// buzzer = 0;
		// // circle_type = CIRCLE_LEFT_RUNNING;
		// // circle_type = CIRCLE_RIGHT_RUNNING;
		// if(thread_end == true) {
		// 	std::cout << "等待其余进程结束" << std::endl;
		// 	usleep(100000);	//等待其余进程结束 100ms
		// 	std::cout << "main() : 退出." << std::endl;
		// 	return 0;
		// }
	}
	return 0;
}

void debug_show() {
	// 显示赛道识别类型
	int aim_p[2];
	map_inv(rptsn[clip(round(aim_distance / sample_dist), 0, rptsn_num - 1)], aim_p);
	line(imageCorrect, Point(0, aim_p[1]), Point(319, aim_p[1]), Scalar(80, 127, 255), 2);

	circle(imageCorrect, Point(COLSIMAGE / 2 - begin_x, begin_y), 3, Scalar(127, 127, 0));
	circle(imageCorrect, Point(COLSIMAGE / 2 + begin_x, begin_y), 3, Scalar(127, 127, 0));
	
	{
		float _p[2] = {mid_x, mid_y};
		int p[2];
		map_inv(_p, p);
		circle(imageCorrect, Point(p[0], p[1]), 5, Scalar(127, 127, 0), -1);
	}
	// extern int col_whitemax_L;
	// extern int col_whitemax_R;
	// line(imageCorrect, Point(col_whitemax_L, 0), Point(col_whitemax_L, 239), Scalar(255, 0, 0), 2);
	// line(imageCorrect, Point(col_whitemax_R, 0), Point(col_whitemax_R, 239), Scalar(0, 0, 255), 2);
	
	for (int i = 0; i < rptsn_num; i++) {
		int p[2];
		map_inv(rptsn[i], p);
		circle(imageCorrect, Point(p[0], p[1]), 1, Scalar(255, 0, 0), -1);
	}

	if (circle_type != CIRCLE_NONE) {
		putText(imageCorrect, "up_y_e:", Point(150, 200),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
			LINE_AA); // 显示赛道识别类型

		if (circle_type == CIRCLE_LEFT_IN) {
			putText(imageCorrect, std::to_string(circle_rptss[circle_Lpt_rptss_id][1]), Point(230, 200),
				cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
				LINE_AA); // 显示赛道识别类型
		}
		putText(imageCorrect, circle_type_name[circle_type], Point(10, 30),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(255, 255, 0), 1,
			LINE_AA);

		putText(imageCorrect, "radius", Point(150, 170),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
			LINE_AA); // 显示赛道识别类型

		putText(imageCorrect, std::to_string(radius), Point(230, 170),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
			LINE_AA); // 显示赛道识别类型
	}
	else if (cross_type != CROSS_NONE) {
		putText(imageCorrect, cross_type_name[cross_type], Point(10, 30),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 255), 1,
			LINE_AA);

		if (cross_type == CROSS_HALF_LEFT) {
			cv::circle(imageCorrect, Point(far_x11, far_y11), 5, Scalar(0, 0, 255), -1);
			int p[2];
			map_inv(rpts0s[clip(Lpt0_rpts0s_id, 0, rpts0s_num - 1)], p);
			cv::circle(imageCorrect, Point(p[0], p[1]), 5, Scalar(0, 0, 255));
    		
			line(imageCorrect,
				Point(far_x11, far_y11),
				Point(far_x11, ROWSIMAGE - 1), Scalar(0, 0, 255), 2);
			
			for (int i = 0; i < far_rpts0s_num; i++) {
				//int p[2];
				//map_inv(far_rpts0s[i], p);
				cv::circle(imageCorrect, Point(far_ipts0[i][0], far_ipts0[i][1]), 1, Scalar(0, 0, 255), -1);
			}
		}

		if (cross_type == CROSS_HALF_RIGHT) {
			cv::circle(imageCorrect, Point(far_x22, far_y22), 5, Scalar(0, 255, 0), -1);
			
			int p[2];
			map_inv(rpts1s[clip(Lpt1_rpts1s_id, 0, rpts1s_num - 1)], p);
    		
			cv::circle(imageCorrect, Point(p[0], p[1]), 5, Scalar(0, 255, 0));

			if(far_Lpt1_found) {
				line(imageCorrect,
				Point(far_x22, far_y22),
				Point(far_x22, ROWSIMAGE - 1), Scalar(0, 255, 0), 2);
			}else{
				line(imageCorrect,
				Point(far_x22, far_y22),
				Point(far_x22, ROWSIMAGE - 1), Scalar(255, 255, 0), 2);
			}
			
			for (int i = 0; i < far_rpts1s_num; i++) {
				//int p[2];
				//map_inv(far_rpts1s[i], p);
				cv::circle(imageCorrect, Point(far_ipts1[i][0], far_ipts1[i][1]), 1, Scalar(0, 255, 0), -1);
			}
		}
	}
	else if (garage_type != GARAGE_NONE) {
		COUT1(garage_type_name[garage_type]);
		putText(imageCorrect, garage_type_name[garage_type], Point(10, 30),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 125, 70), 1,
			LINE_AA);
	}
	else {
		putText(imageCorrect, "[1] Track", Point(10, 30),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
			LINE_AA); // 显示赛道识别类型
	}

	{
		putText(imageCorrect, "bias_p", Point(10, 60),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
			LINE_AA); // 显示赛道识别类型

		putText(imageCorrect, std::to_string(bias_p), Point(77, 60),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
			LINE_AA); // 显示赛道识别类型
	}

	{
		putText(imageCorrect, "bias_d", Point(10, 80),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
			LINE_AA); // 显示赛道识别类型

		putText(imageCorrect, std::to_string(bias_d), Point(77, 80),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
			LINE_AA); // 显示赛道识别类型
	}

	{
		putText(imageCorrect, "Lr", Point(10, 100),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
			LINE_AA); // 显示赛道识别类型

		putText(imageCorrect, std::to_string((long long)Lradius), Point(30, 100),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
			LINE_AA); // 显示赛道识别类型
	}

	{
		putText(imageCorrect, "Rr", Point(80, 100),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
			LINE_AA); // 显示赛道识别类型

		putText(imageCorrect, std::to_string((long long)Rradius), Point(100, 100),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
			LINE_AA); // 显示赛道识别类型
	}

	{
		putText(imageCorrect, "Lr_F", Point(10, 120),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
			LINE_AA); // 显示赛道识别类型

		putText(imageCorrect, std::to_string((long long)Lradius_far), Point(60, 120),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
			LINE_AA); // 显示赛道识别类型
	}

	{
		putText(imageCorrect, "Rr_F", Point(110, 120),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
			LINE_AA); // 显示赛道识别类型

		putText(imageCorrect, std::to_string((long long)Rradius_far), Point(160, 120),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
			LINE_AA); // 显示赛道识别类型
	}

	{
		putText(imageCorrect, "midAdd", Point(10, 155),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
			LINE_AA); // 显示赛道识别类型

		putText(imageCorrect, std::to_string(midAdd), Point(90, 155),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
			LINE_AA); // 显示赛道识别类型

		line(imageCorrect,
			Point(COLSIMAGE / 2 - 30 * ((midAdd - Servo_Center_Mid) / 900), ROWSIMAGE - 10),
			Point(COLSIMAGE / 2 - 30 * ((midAdd - Servo_Center_Mid) / 900), ROWSIMAGE), Scalar(0, 0, 255), 5);
		line(imageCorrect,
			Point(COLSIMAGE / 2, ROWSIMAGE - 30),
			Point(COLSIMAGE / 2, ROWSIMAGE), Scalar(0, 255, 0), 2);
	}

	if (is_straight0) {
		putText(imageCorrect, "LStraight", Point(10, 235),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
			LINE_AA); // 显示赛道识别类型
	}
	else {}
	if (is_straight1) {
		putText(imageCorrect, "RStraight", Point(230, 235),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
			LINE_AA); // 显示赛道识别类型
	}
	else {}
	
	if (abs(Servo_Center_Mid - midAdd) > BIAS_SPEED_LIMIT) {
		if(track_state == TRACK_STATE::TRACK_SLOW) {
				putText(imageCorrect, "SPEED_SLOWEST", Point(100, 235),
					cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(255, 0, 255), 1,
					LINE_AA); // 显示赛道识别类型
		}
	}
	else{
		if (is_straight0 && is_straight1) {
		}
		else {
			putText(imageCorrect, "SStraight", Point(100, 235),
					cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(255, 0, 255), 1,
					LINE_AA); // 显示赛道识别类型

		}
	}

	{
		putText(imageCorrect, "TS::S:", Point(10, 140),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(255, 57, 90), 1,
			LINE_AA); // 显示赛道识别类型
		putText(imageCorrect, std::to_string((int)slow_count), Point(65, 140),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(255, 57, 90), 1,
			LINE_AA); // 显示赛道识别类型

		putText(imageCorrect, "TS::F:", Point(110, 140),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(255, 57, 90), 1,
			LINE_AA); // 显示赛道识别类型
		putText(imageCorrect, std::to_string((int)fast_count), Point(165, 140),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(255, 57, 90), 1,
			LINE_AA); // 显示赛道识别类型
	}

	putText(imageCorrect, "speed", Point(190, 30),
		cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
		LINE_AA); // 显示赛道识别类型

	putText(imageCorrect, std::to_string(speed_set), Point(250, 30),
		cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
		LINE_AA); // 显示赛道识别类型

	putText(imageCorrect, "sL", Point(190, 50),
		cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
		LINE_AA); // 显示赛道识别类型

	putText(imageCorrect, std::to_string(stdevLeft), Point(250, 50),
		cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
		LINE_AA); // 显示赛道识别类型

	putText(imageCorrect, "sR", Point(190, 70),
		cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
		LINE_AA); // 显示赛道识别类型

	putText(imageCorrect, std::to_string(stdevRight), Point(250, 70),
		cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
		LINE_AA); // 显示赛道识别类型

	putText(imageCorrect, std::to_string(rpts0s_num), Point(40, 220),
		cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(255, 125, 0), 1,
		LINE_AA); // 显示赛道识别类型

	putText(imageCorrect, std::to_string(rpts1s_num), Point(250, 220),
		cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(255, 125, 0), 1,
		LINE_AA); // 显示赛道识别类型


	if (Lpt0_found) {
		putText(imageCorrect, std::to_string(Lpt0_rpts0s_id), Point(80, 220),
			cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
			LINE_AA); // 显示赛道识别类型
	}
	if (Lpt1_found) {
		putText(imageCorrect, std::to_string(Lpt1_rpts1s_id), Point(210, 220),
			cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
			LINE_AA); // 显示赛道识别类型
	}
	


	putText(imageCorrect, "FPS", Point(10, 175),
		cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
		LINE_AA); // 显示赛道识别类型

	putText(imageCorrect, std::to_string(FPS), Point(100, 175),
		cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
		LINE_AA); // 显示赛道识别类型

	for (int i = 0; i < ipts0_num; i++) {
		int c = ipts0[i][0];
		int r = ipts0[i][1];
		MAT_AT_SET(imageCorrect, r, c, 0, 0, 255);

	}

	for (int i = 0; i < ipts1_num; i++) {
		int c = ipts1[i][0];
		int r = ipts1[i][1];
		MAT_AT_SET(imageCorrect, r, c, 0, 255, 0);
	}

	for (int i = 0; i < circle_ipts_num; i++) {
		int c = circle_ipts[i][0];
		int r = circle_ipts[i][1];
		//MAT_AT_SET(imageCorrect, r, c, 0, 125, 125);
		cv::circle(imageCorrect, Point(c, r), 1, Scalar(0, 215, 255));
	}
}

void* frame_get(void* arg) {
	cap_mutex = (pthread_mutex_t*)arg;
	cpu_set_t mask;	// cpu核的集合
    // cpu_set_t get;	// 获取在集合中的cpu
    
	CPU_ZERO(&mask);	// 将集合置为空集
    CPU_SET(0, &mask);	// 设置亲和力值 cpu 0
    CPU_SET(1, &mask);	// 设置亲和力值 cpu 1
	CPU_SET(2, &mask);	// 设置亲和力值 cpu 2
    CPU_SET(3, &mask);	// 设置亲和力值 cpu 3

	// 设置线程cpu亲和力
    if(sched_setaffinity(0,sizeof(cpu_set_t),&mask)==-1){
        printf("frame_get warning: could not set CPU affinity, continuing...\n");
    }


	unsigned char *yuv422frame = NULL;
	unsigned long yuvframeSize = 0;

	string videoDev = "/dev/video0";
	V4L2Capture *vcap = new V4L2Capture(const_cast<char*>(videoDev.c_str()), 640, 480);
	vcap->openDevice();
	vcap->initDevice();
	vcap->startCapture();


	// VideoCapture cap("/dev/video0", cv::CAP_V4L2);
	// // VideoCapture cap(cv::CAP_V4L2 + 0);
	// std::string video = "lx.mp4";
	// video = "../res/video/2024.mp4";
	// // VideoCapture cap(video);
	// cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));//视频流格式
	// cap.set(CV_CAP_PROP_FPS, 120);//帧率
	// cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	// cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

	// cap.set(cv::CAP_PROP_BRIGHTNESS, 0);		// 亮度0
	// cap.set(cv::CAP_PROP_CONTRAST, 32);		// 对比度32
	// cap.set(cv::CAP_PROP_SATURATION, 60);	// 饱和度64
	// cap.set(cv::CAP_PROP_HUE, 0);			// 色调0
	// // cap.set(cv::CAP_PROP_EXPOSURE, -7);		// 曝光-7
	// cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.75);	// 0.75自动曝光 0.25手动曝光


	IplImage* img;
	CvMat cvmat;

    while(1) {
		vcap->getFrame((void **) &yuv422frame, (size_t *)&yuvframeSize);
		cvmat = cvMat(480, 640, CV_8UC3,(void*)yuv422frame);		//CV_8UC3

		//解码
		img = cvDecodeImage(&cvmat,1);
		if(!img) {
			printf("DecodeImage error!\n");
			continue;
		}
		frame = cv::cvarrToMat(img);  

		// cap.read(frame);
		// cap >> frame;
		if(frame.empty())continue;
		cv::resize(frame, frame, cv::Size(320, 240));

		// 定义覆盖车头的多边形的顶点坐标
		vector<cv::Point> pts;
		pts.push_back(cv::Point(103, 239));
		pts.push_back(cv::Point(103, 236));
		pts.push_back(cv::Point(101, 232));
		pts.push_back(cv::Point(101, 224));
		pts.push_back(cv::Point(112, 205));
		pts.push_back(cv::Point(128, 188));
		pts.push_back(cv::Point(146, 183));
		pts.push_back(cv::Point(176, 183));


		pts.push_back(cv::Point(184, 184));
		pts.push_back(cv::Point(211, 197));
		pts.push_back(cv::Point(224, 218));
		pts.push_back(cv::Point(227, 224));
		pts.push_back(cv::Point(230, 239));
		// pts.push_back(cv::Point(103, 239));
	

		vector<vector<cv::Point>> contours;
		contours.push_back(pts);
		drawContours(frame, contours, -1, Scalar(255, 255, 255), -1);
		// drawContours(frame, contours, -1, Scalar(213, 248, 251), -1);
		// drawContours(frame, contours, -1, Scalar(189, 228, 230), -1);

		if(!ai_run) {
			pthread_mutex_lock(&mutex_ai);
			ai_frame = frame.clone();
			pthread_mutex_unlock(&mutex_ai);
		}

		if(!line_run) {
			pthread_mutex_lock(&mutex_line);
			rgb_frame = frame.clone();
			pthread_mutex_unlock(&mutex_line);
		}
		if(thread_end) {
			std::cout << "frame_get 线程等待." << std::endl;
			usleep(5000);	//等待其余线程关闭
			std::cout << "thread() : frame_get 退出." << std::endl;
			vcap->stopCapture();
			vcap->freeBuffers();
			vcap->closeDevice();

			return NULL;
		}

		cvReleaseImage(&img);
		vcap->backFrame();
    }
	vcap->stopCapture();
	vcap->freeBuffers();
	vcap->closeDevice();

    return NULL;
}

void* trackline_process(void* arg) {
	line_mutex = (pthread_mutex_t*)arg;
	line_init = true;

	cpu_set_t mask;	// cpu核的集合
    // cpu_set_t get;	// 获取在集合中的cpu
    
	CPU_ZERO(&mask);	// 将集合置为空集
    CPU_SET(2, &mask);	// 设置亲和力值 cpu 2
    CPU_SET(3, &mask);	// 设置亲和力值 cpu 3
    
	// 设置线程cpu亲和力
    if(sched_setaffinity(0,sizeof(cpu_set_t), &mask)==-1) {
        printf("line_process warning: could not set CPU affinity, continuing...\n");
    }

    while(1) {
		// circle_type = CIRCLE_LEFT_RUNNING;
		// circle_type = CIRCLE_RIGHT_RUNNING;
        // CPU_ZERO(&get);
		// // 获取线程cpu亲和力
        // if(sched_getaffinity(0,sizeof(get),&get)==-1){
        //     printf("warning: could not get thread affinity, continuing...\n");
        // }
		// 判断线程与哪个cpu有亲和力
		if(line_init /*&& (CPU_ISSET(2,&get) || CPU_ISSET(3,&get))*/ && !rgb_frame.empty()) {
			lineFrame = Mat::zeros(cv::Size(320, 240), CV_8UC1);
			nitoushi = Mat::zeros(cv::Size(320, 240), CV_8UC1);

			buzzer = 0;
			Mat frame = rgb_frame.clone();
			pthread_mutex_lock(line_mutex);
			line_run = true;
			pthread_mutex_unlock(line_mutex);

			// 处理帧时长监测
			static auto preTime = std::chrono::duration_cast<std::chrono::milliseconds>(
									std::chrono::system_clock::now().time_since_epoch())
									.count();
			auto startTime = std::chrono::duration_cast<std::chrono::milliseconds>(
								std::chrono::system_clock::now().time_since_epoch())
								.count();
			// std::cout << "line time : " << startTime - preTime << "ms" << std::endl;
			line_time = startTime - preTime;
			FPS = (double)1000.0 / (startTime - preTime);
			preTime = startTime;

			cvtColor(frame, frame, COLOR_BGR2GRAY);
			frameTOimg_raw(frame);
			thres = threshold(frame, frame, 0, 255, THRESH_OTSU);
			// 车库斑马线检查(斑马线优先级高，最先检查)
			bool rescue_it = (rescue_state != 10 && rescue_state != 20 && rescue_state != 30);
			bool danger_it = (danger.state == 0 || danger.state == 1);
			bool racing_it = (racing.state == 0 || racing.state == 1);
			printf("rescue_it:%d\tdanger_it:%d\tracing_it:%d\r\n", rescue_it, danger_it, racing_it);
			if(rescue_it && danger_it && racing_it)
				check_garage();
			
			if(garage_type != GARAGE_NONE){
				begin_y = 125;
			}else{
				begin_y = BEGIN_Y;
			}

			get_normal_line();

			line_process();

			// find_corners();
		
			if (circle_type != CIRCLE_NONE) {
				switch (circle_type) {
					case CIRCLE_LEFT_BEGIN:
					case CIRCLE_RIGHT_BEGIN:
						buzzer = 0;
						speed.normal = SPEED_CIRCLE_BEGIN;
						// mid_x = MIDX_MID;
						break;
					case CIRCLE_LEFT_IN:
					case CIRCLE_RIGHT_IN:
						speed.normal = speed.slow = SPEED_CIRCLE_MIN;
						buzzer = 1;
						// speed.normal = 21;
						break;
					case CIRCLE_LEFT_RUNNING:
						speed.normal = speed.slow = SPEED_CIRCLE_RUNNING;
						// aim_distance = 0.42;
						// mid_x = 161.7;
						break;
					case CIRCLE_RIGHT_RUNNING:
						speed.normal = speed.slow = SPEED_CIRCLE_RUNNING;
						// aim_distance = 0.42;
						// mid_x = 159.3;
						break;
					case CIRCLE_LEFT_OUT:
					case CIRCLE_RIGHT_OUT:
						fast_count = 0;
						slow_count = 0;
						speed.normal = speed.slow = SPEED_ADD;
						buzzer = 1;
						break;
					default:
						break;
				}
			}
			else if (cross_type != CROSS_NONE) {
				//aim_distance = 0.4;
				switch(cross_type){
					case CROSS_BEGIN:
						buzzer = 1;
						break;
					case CROSS_HALF_LEFT:
					case CROSS_HALF_RIGHT:
						buzzer = 1;
						break;
					default:
						break;
				}
			}
			else {
				speed.normal = SPEED_NORMAL;
				speed.fast = SPEED_FAST;
				speed.slow = SPEED_SLOW;
				speed.add = SPEED_ADD;
				if(!DANGER_ENABLE && !RACING_ENABLE)
					mid_x = MIDX_MID;
				

				// speed.normal = speed.fast = speed.slow = speed.add = 28;

				aim_distance = AIM_DISTANCE;
			}

			track_switch();
			
			// 分别检查十字 三叉 和圆环, 十字优先级最高
			if (garage_type == GARAGE_NONE){
				check_cross();
			}
			if (!DANGER_ENABLE && !RACING_ENABLE && !RESCUE_ENABLE && garage_type == GARAGE_NONE && cross_type == CROSS_NONE)
				check_circle();
			if (cross_type != CROSS_NONE) {
				circle_type = CIRCLE_NONE;
			}
			//根据检查结果执行模式
			if (cross_type != CROSS_NONE) run_cross();
			if (circle_type != CIRCLE_NONE) run_circle();
			if (garage_type != GARAGE_NONE) run_garage();

			controlCenter.smotorSolution(danger.state, racing.state);

			if (is_start) {
				if (start_speed_count == 0) {
					speed_mode = SPEED_MODE::NORMAL;
					int count = 0;
					if(it_AI) {
						if(it_DANGER)count++;					
						if(it_RESCUE)count++;					
						if(it_RACING)count++;					
						if(it_BRIDGE)count++;
					}

					if(it_AI) {
						driver->carControl(0, Servo_Center_Mid, 0);
						waitKey(250);
						driver->carControl(0, Servo_Center_Mid, 1);
						waitKey(300);
						for(int i = 0; i < count; i++) {
							driver->carControl(0, Servo_Center_Mid, 0);
							waitKey(50);
							driver->carControl(0, Servo_Center_Mid, 1);
							waitKey(50);						
						}
					}else{
						for(int i = 0; i < 3; i++) {
							driver->carControl(0, Servo_Center_Mid, 0);
							waitKey(50);
							driver->carControl(0, Servo_Center_Mid, 1);
							waitKey(50);						
						}
					}
					driver->carControl(0, Servo_Center_Mid, 0);
					waitKey(3000);
				}
				driver->carControl(speed.slow, midAdd, buzzer); // 串口通信，姿态与速度控制
				//if (start_speed.count >= speed.slow) {
					is_start = false;
					stop = false;
				//}
			}
			else if (stop) {
				speed_mode = SPEED_MODE::NORMAL;
				static int stop_speed_count = speed.slow;
				const int stop_num = 1;
				static int cur_stop_num = 0;
				if (stop_speed_count >= 0) {
					if(cur_stop_num++ < stop_num) {
						driver->carControl(stop_speed_count, midAdd, 1);
					}
					// 执行一次减速
					else{
						cur_stop_num = 0;
						stop_speed_count-=2;
					}
				}else{
					driver->carControl(0, midAdd, 1);
					waitKey(50);
					driver->carControl(0, midAdd, 0);
					waitKey(50);
					driver->carControl(0, midAdd, 1);
					driver->carControl(0, midAdd, 1);
					driver->carControl(0, midAdd, 1);
					driver->carControl(0, midAdd, 1);
					driver->carControl(0, midAdd, 1);
					driver->carControl(0, midAdd, 1);
					waitKey(200);
					driver->carControl(0, Servo_Center_Mid, 0);
					waitKey(50);
					driver->carControl(0, Servo_Center_Mid, 1);
					waitKey(50);
					driver->carControl(0, Servo_Center_Mid, 0);
					COUT1(">>>>>>>>>>>>>>>>");
					COUT1("入库 >>>>>> 停车");
					thread_end = true;
					std::cout << "thread() : trackline_process 退出." << std::endl;
					return NULL;
				}
			}
			else {
				if(AI_ENABLE) {
					// 危险区
					if(DANGER_ENABLE){
						speed_pid_mode = 1;
						COUT1("DANGER_ENABLE");
						speed_mode = SPEED_MODE::NORMAL;
						int speed_target = SPEED_DANGER;
						static int danger_slow_count = 0;
						if(danger.state == 1) {
							danger_slow_count = 0;
						}
						if(danger_slow_count++ > 20) {
							driver->carControl(speed_target, midAdd, buzzer);
						}else{
							COUT1("DANGER_SLOWWWWW");
							driver->carControl(SPEED_DANGER, midAdd, buzzer);							
						}
					}
					// 救援区
					else if(RESCUE_ENABLE) {
						speed_pid_mode = 1;
						speed_mode = SPEED_MODE::NORMAL;
						if(rescue_state == 10) {
							mid_x = MIDX_MID;
							driver->carControl(SPEED_RESCUE, 5450, buzzer);
						}else if(rescue_state == 20) {
							mid_x = MIDX_MID;
							driver->carControl(SPEED_RESCUE, 3650, buzzer);
						}else if(rescue_state == 30){
							AI_ENABLE = RESCUE_ENABLE = false;
							driver->carControl(SPEED_RESCUE, midAdd, buzzer);
						}else{
							driver->carControl(18, midAdd, buzzer);						
						}
					}
					// 追逐区
					else if(RACING_ENABLE){
						speed_pid_mode = 1;
						// ai_state = racing.state; 
						COUT2("RACING_ENABLE", racing.state);
						speed_mode = SPEED_MODE::NORMAL;
						int speed_target = SPEED_RACING;
						static int racing_slow_count = 0;

						// 发现车辆，执行默认减速
						if(racing.state == 1) {
							racing_slow_count = 0;
							// if(racing_slow_count++ < 5) {
							// 	speed_pid_mode = 2;
							// }
							// else if(racing_slow_count > 50) {
							// 	racing_slow_count = 0;
							// }
						}
						else if(racing.state == 2 || racing.state == 3 || racing.state == 4) {
							// racing_slow_count = 0;
						}
						// 常规默认避让，检查标签中
						else if(racing.state == 100 || racing.state == -100) {
							speed_target = 15;
						}
						// 20safe左障碍 -20safe右障碍 
						else if(racing.state == 20 || racing.state == -20) {
							speed_target = 25;
						}
						// 330danger左撞车 -330danger右撞车
						else if(racing.state == 330 || racing.state == -330) {
							speed_target = 15;
						}
						// 拦截绕行
						else if(racing.state == 400 || racing.state == 40 || racing.state == -40) {
							speed_target = 15;
						}
						else if(racing.state == 440 || racing.state == -440) {
							speed_target = 15;
						}
						// 4000sus左挡车
						else if(racing.state == 4000) {
							speed_target = 0;
							// midAdd = 4050;
						}
						// -4000sus右挡车
						else if(racing.state == -4000) {
							speed_target = 0;
							// midAdd = 4850;
						}

						if(racing_slow_count++ > 25) {
							driver->carControl(speed_target, midAdd, buzzer);
						}else{
							COUT1("RACING_SLOWWWWW");
							driver->carControl(SPEED_RACING, midAdd, buzzer);							
						}

						// driver->carControl(speed_target, midAdd, buzzer);
					}
				} else {
					speed_mode = SPEED_MODE::NORMAL;
					static int out_count = 0;
					// 急减速控制
					if (rpts0s_num <= 1 && rpts1s_num <= 1 && cross_type == CROSS_NONE && circle_type == CIRCLE_NONE) {
						if (++out_count > OUT_COUNT) {
							speed_mode = SPEED_MODE::NORMAL;
							driver->carControl(0, Servo_Center_Mid, 1);
							driver->carControl(0, Servo_Center_Mid, 1);
							driver->carControl(0, Servo_Center_Mid, 1);
							driver->carControl(0, Servo_Center_Mid, 1);
							driver->carControl(0, Servo_Center_Mid, 1);
							waitKey(200);
							driver->carControl(0, Servo_Center_Mid, 0);
							waitKey(50);
							driver->carControl(0, Servo_Center_Mid, 1);
							waitKey(50);
							driver->carControl(0, Servo_Center_Mid, 0);
							waitKey(50);
							driver->carControl(0, Servo_Center_Mid, 1);
							waitKey(50);
							driver->carControl(0, Servo_Center_Mid, 0);
							COUT1("!!!!!!!!!!!!!!!!!");
							COUT1("出界 >>>>>> 停车");
							waitKey(100);
							thread_end = true;
							std::cout << "thread() : trackline_process 退出." << std::endl;
							return NULL;
						}else{
							driver->carControl(SPEED_SLOWEST, midAdd, buzzer);
						}
					}
					else {
						out_count = 0;
						// 减速状态
						if(circle_type == CIRCLE_NONE && track_state == TRACK_STATE::TRACK_SLOW) {
							speed.fast = speed.normal = speed.slow = speed.add = SPEED_SLOWEST;
							if(!BRIDGE_ENABLE)buzzer = 1;
						}
						
						if(garage_type == GARAGE_FOUND) {
							speed.fast = speed.normal = speed.slow = speed.add = 20;
						}

						if(BRIDGE_ENABLE) {
							COUT2("BRIDGE_ENABLE", bridge.state);
							buzzer = 1;
							speed.fast = speed.normal = speed.slow = speed.add = SPEED_BRIDGE;
						}

						if(RACING_FIND) {
							COUT2("RACING_FIND", racing.state);
							speed.fast = speed.normal = speed.slow = speed.add = SPEED_RACING;
						}

						if (abs(Servo_Center_Mid - midAdd) > BIAS_SPEED_LIMIT) {
							speed_mode = SPEED_MODE::NORMAL;
							if (circle_type == CIRCLE_LEFT_END || circle_type == CIRCLE_RIGHT_END) {
								driver->carControl(speed.add, midAdd, buzzer); // 串口通信，姿态与速度控制
							}
							else {
								driver->carControl(speed.slow, midAdd, buzzer); // 串口通信，姿态与速度控制
							}
						}
						else {
							speed_mode = SPEED_MODE::NORMAL;
							if (is_straight0 && is_straight1) {
								driver->carControl(speed.fast, midAdd, buzzer); // 串口通信，姿态与速度控制
							}
							else {
								driver->carControl(speed.normal, midAdd, buzzer); // 串口通信，姿态与速度控制
							}
						}
					}
				}
			}
	
			pthread_mutex_lock(line_mutex);
			rgb_frame = cv::Mat();
			line_run = false;
			pthread_mutex_unlock(line_mutex);
		}

		if(thread_end) {
			std::cout << "thread() : trackline_process 退出." << std::endl;
			return NULL;
		}
    }
    return NULL;
}	