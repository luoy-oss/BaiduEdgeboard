#include "main.h"
#include "../code/headfile.h"

#include "../code/common.h"
#include "../code/pid.h"

#include "../code/debugger.h"
#include "../code/imgproc.h"
#include "../code/camera_param.h"

#include "../recognition/track_recognition.h"
#include "../recognition/cross_recognition.h"
#include "../recognition/circle_recognition.h"
#include "../recognition/garage_recognition.h"

#ifdef _WIN32
#include "SerialPort.h"
#else
#include "../code/uart.h"
#endif

using namespace cv;


//�������ƫ��
float angle;

Mat frame;			//RGBͼ��
Mat lineFrame;		//ԭʼ����Ѱ��
Mat nitoushi;		//��͸��������

Mat imageCorrect;	//����Ԫ��


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
float begin_y = 162;

float line_blur_kernel = 7;//ԭ7
float pixel_per_meter = 94;//ƽ�����أ��������
float sample_dist = 0.02;
float angle_dist = 0.2;
float far_rate = 0.5;
float aim_distance = 0.68;
bool adc_cross = false;

//����������extern pid_param_t servo_pid;
pid_param_t servo_pid(1.5, 0, 1.0, 0.8, 15, 5, 15);


#ifdef _WIN32
std::shared_ptr<CSerialPort> driver = nullptr; // ��ʼ����������
//���ڷ��ͺ���Z
void send(unsigned char temp[8]) {
	driver->WriteData(temp, 8);//����������Ǹ����ڷ������ݵĺ�����temp����Ҫ���͵����顣
	temp[5] = 0;
}
#else
std::shared_ptr<Driver> driver = nullptr; // ��ʼ����������
#endif

long double bias_p = 0.0;
long double bias_p_last = 0.0;
long double bias_i = 0.0;
long double bias_d = 0.0;
long double ctr = 0.0;
long double midAdd = 0.0;
long double lastmidAdd = 0.0;
double maxmA = -2333333;
double minmA = 2333333;
void debug_show();

//1. �ṹ�����Ͷ���
typedef struct
{
	float LastP;    //�ϴι���Э���� ��ʼ��ֵΪ0.02     --e(ESTk-1)  �ϴ�Э����
	float Now_P;    //��ǰ����Э���� ��ʼ��ֵΪ0        --Ԥ��e(ESTk) ��ǰ����Э����
	float out;      //�������˲������ ��ʼ��ֵΪ0
	float Kg;       //���������� ��ʼ��ֵΪ0             --Kk
	float Q;        //��������Э���� ��ʼ��ֵΪ0.1
	float R;        //�۲�����Э���� ��ʼ��ֵΪ0.543        --e(MEAk)  �������
}KFP;

static KFP KFP_height = { 0.02,0,1500,0,0.5,0.543 };


/**
 *�������˲���
 *@param KFP *kfp �������ṹ�����
 *   float input ��Ҫ�˲��Ĳ����Ĳ���ֵ�����������Ĳɼ�ֵ��
 *@return �˲���Ĳ���������ֵ��
 */
float kalmanFilter(KFP* kfp, float input) {
	//Ԥ��Э����̣�kʱ��ϵͳ����Э���� = k-1ʱ�̵�ϵͳЭ���� + ��������Э����
	kfp->Now_P = kfp->LastP + kfp->Q;

	//���������淽�̣����������� = k1-1ʱ��ϵͳ����Э���� / ��kʱ��ϵͳ����Э���� + �۲�����Э���
	kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
	//��������ֵ���̣�kʱ��״̬����������ֵ = ״̬������Ԥ��ֵ + ���������� * ������ֵ - ״̬������Ԥ��ֵ��
	kfp->out = kfp->out + kfp->Kg * (input - kfp->out);//��Ϊ��һ�ε�Ԥ��ֵ������һ�ε����ֵ
	//����Э�����: ���ε�ϵͳЭ����� kfp->LastP ����һ������׼����
	kfp->LastP = (1 - kfp->Kg) * kfp->Now_P;

	return kfp->out;
}


int main() {
#ifndef _WIN32
	// USBת���ڵ��豸��Ϊ /dev/ttyUSB0
	driver = std::make_shared<Driver>("/dev/ttyUSB0", BaudRate::BAUD_115200);
	if (driver == nullptr) {
		std::cout << "Create Uart-Driver Error!" << std::endl;
		return -1;
	}
	// ���ڳ�ʼ�����򿪴����豸�����ô������ݸ�ʽ
	int ret = driver->open();
	if (ret != 0) {
		std::cout << "Uart Open failed!" << std::endl;
		return -1;
	}
#else
	//�Ƿ�򿪴��ڣ�3�������������ӵ��Ե�com�ڣ��������豸�������鿴��Ȼ������������
	if (!driver->InitPort(7, CBR_115200, 'N', 8, 1, EV_RXCHAR)) {
		std::cout << "initPort fail !" << std::endl;
	}
	else {
		std::cout << "initPort success !" << std::endl;
	}
#endif

	VideoCapture cap(0);
	std::string video = "lx.mp4";
	//video = "garage.mp4";
	//VideoCapture cap(video);
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
		static int i = 1;
		std::string filename = "../frame/" + std::to_string(i++) + ".jpg";
		imwrite(filename, frame);
#ifdef CAR_DEBUG
		imageCorrect = frame.clone();
#endif 

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
		//aim_distance = 0.68;

		if (circle_type != CIRCLE_NONE) {
			aim_distance = 0.47;
		}
		else if (cross_type != CROSS_NONE) {
			aim_distance = 0.45;
		}
		else {
			aim_distance = 0.5;
		}

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

		//std::cout<<track_type<<"\t"<<rpts0s_num<<","<<rpts1s_num<<std::endl;

		// ��������߼��(���������ȼ��ߣ����ȼ��)
		check_garage();

		// �ֱ���ʮ�� ���� ��Բ��, ʮ�����ȼ����
		if (garage_type == GARAGE_NONE)
			check_cross();
		if (garage_type == GARAGE_NONE && cross_type == CROSS_NONE)
			check_circle();
		if (cross_type != CROSS_NONE) {
			circle_type = CIRCLE_NONE;
		}
		//���ݼ����ִ��ģʽ
		if (cross_type != CROSS_NONE) run_cross();
		if (circle_type != CIRCLE_NONE) run_circle();
		if (garage_type != GARAGE_NONE) run_garage();

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
		{
			// ���ֶ�Ӧ��(��������ʼ��)
			// for (int r = 0; r < ROWSIMAGE; r++) {
			// 	if ((int)mapx[COLSIMAGE / 2][r] == 161) {
					// COUT2("r:" + to_string(r),mapy[COLSIMAGE / 2][r]);
			// 	}
			// }
			float cx = mapx[COLSIMAGE / 2][164];
			float cy = 230;
			// draw_x(&img_line, cx, cy, 2, 255);
			// �������(��ʼ�����߹�һ��)
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

			// ����ģʽ�£����������(���ڱ��߻���һȦ���������������Ϊ�������һ���㣬�Ӷ������޷���������)
			if (garage_type == GARAGE_IN || cross_type == CROSS_IN) begin_id = 0;

			// �����е㣬ͬʱ����㲻����󼸸���
			if (begin_id >= 0 && rpts_num - begin_id >= 3) {



				// ��һ������
				rpts[begin_id][0] = cx;
				rpts[begin_id][1] = cy;
				rptsn_num = sizeof(rptsn) / sizeof(rptsn[0]);
				resample_points(rpts + begin_id, rpts_num - begin_id, rptsn, &rptsn_num, sample_dist * pixel_per_meter);

				// ԶԤê��λ��
				int aim_idx = clip(begin_id + round(aim_distance / sample_dist), 0, rptsn_num - 1);
				// ��Ԥê��λ��
				int aim_idx_near = clip(begin_id + round(0.25 / sample_dist), 0, rptsn_num - 1);

				draw_o(&img_line, rptsn[begin_id][0], rptsn[begin_id][1], 3, 255);
				// ����Զê��ƫ��ֵ
				float dx = rptsn[aim_idx][0] - cx;
				float dy = cy - rptsn[aim_idx][1] + 0.2 * pixel_per_meter;
				float dn = sqrt(dx * dx + dy * dy);
				float error = -atan2f(dx, dy) * 180 / PI;
				assert(!isnan(error));

				// �������㷨(ֻ����Զ��)
				long double pure_angle = atanl(pixel_per_meter * 2 * 0.2 * dx / dn / dn) / PI * 180 / 1.0;
				// COUT1(pure_angle);
				midAdd = (long double)(pure_angle * (125.0 / 22.5));
				if (midAdd > 125) {
					midAdd = 125;
				}
				else if (midAdd < -125) {
					midAdd = -125;
				}

				bias_p = rptsn[aim_idx][0] - cx;
				bias_d = bias_p - bias_p_last;
				bias_p_last = bias_p;
				// bias_i = 0.0;
				for (int i = begin_id; i < aim_idx; i++) {
					bias_i += rptsn[aim_idx][0] - cx;
				}

				static long double Kp = 8.3;
				static long double Ki = 0.0007;
				static long double Kd = 20;

				// if(abs(bias_p) < 11){
				// 	ctr = (long double)bias_p * 5.5 + (long double)bias_i * Ki + (long double)bias_d * Kd;
				// }else{
				// 	ctr = (long double)bias_p * 9.5/*9.5*/ + (long double)bias_i * Ki + (long double)bias_d * Kd;  
				// }

				long double P = (bias_p * bias_p) / 65 + 4.3;
				// COUT1(P);
				ctr = (long double)bias_p * P/*9.5*/ + (long double)bias_i * Ki + (long double)bias_d * Kd;
				midAdd = 1500 - ctr;
				// midAdd = kalmanFilter(&KFP_height, midAdd);
				if (midAdd > 1800) {
					midAdd = 1800;
				}
				if (midAdd < 1200) {
					midAdd = 1200;
				}
				midAdd *= 3;

				// bias_p = rptsn[aim_idx][0] - cx;
				// bias_i += bias_p;
				// bias_d = bias_p - bias_p_last;
				// bias_p_last = bias_p;

				// midAdd = bias_p;
				// if (midAdd > 40) {
				// 	midAdd = 40;
				// }else if (midAdd < -40){
				// 	midAdd = -40;
				// }
			}
			else {
				// ���ߵ����(��������)���򲻿��ƶ��
				rptsn_num = 0;
			}

		}

		draw_circle();
		draw_cross();
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

#ifdef CAR_DEBUG
		// static int i = 0;
		// std::string file = "./frame/" + std::to_string(++i) + ".jpg";
		// imwrite(file, imageCorrect);

		debug_show();
		show_line();
		imshow("frame", frame);
		waitKey(1);
#endif

#ifndef _WIN32
		// driver->carControl(1, midAdd);	
			// if(midAdd > 5){
			// 	driver->carControl(2, midAdd); // ����ͨ�ţ���̬���ٶȿ���
			// }else{
			// 	if(is_straight0 && is_straight1){
			// 		driver->carControl(3, midAdd); // ����ͨ�ţ���̬���ٶȿ���
			// 	}
			// 	else{
			// 		driver->carControl(1, midAdd); // ����ͨ�ţ���̬���ٶȿ���
			// 	}
			// }

			//int speed = speed_control();
			//driver->carControl(speed, midAdd);
		if (abs(4500 - midAdd) > 500) {
			if (circle_type == CIRCLE_LEFT_END || circle_type == CIRCLE_RIGHT_END) {
				driver->carControl(1, midAdd); // ����ͨ�ţ���̬���ٶȿ���
			}
			else {
				driver->carControl(2, midAdd); // ����ͨ�ţ���̬���ٶȿ���
			}
		}
		else {
			if (is_straight0 && is_straight1) {
				driver->carControl(3, midAdd); // ����ͨ�ţ���̬���ٶȿ���
			}
			else {
				driver->carControl(1, midAdd); // ����ͨ�ţ���̬���ٶȿ���
			}
		}

#else
		unsigned char sendData[8];
		sendData[0] = 0x23;
		sendData[2] = 0x00;
		sendData[3] = 4;
		sendData[4] = 0x00;
		sendData[5] = 0x00;
		sendData[6] = 0x00;
		sendData[7] = 0x21;
		driver->WriteData(sendData, 8);//����������Ǹ����ڷ������ݵĺ�����sendData����Ҫ���͵�����
#endif
	}
	return 0;
}

void debug_show() {
	// ��ʾ����ʶ������
	if (circle_type != CIRCLE_NONE) {
		putText(imageCorrect, circle_type_name[circle_type], Point(10, 30),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(255, 255, 0), 1,
			LINE_AA);
	}
	else if (cross_type != CROSS_NONE) {
		putText(imageCorrect, cross_type_name[cross_type], Point(10, 30),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 255), 1,
			LINE_AA);
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
			LINE_AA); // ��ʾ����ʶ������
	}

	{
		putText(imageCorrect, "bias_p", Point(10, 60),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
			LINE_AA); // ��ʾ����ʶ������

		putText(imageCorrect, std::to_string(bias_p), Point(100, 60),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
			LINE_AA); // ��ʾ����ʶ������
	}

	{
		putText(imageCorrect, "bias_i", Point(10, 90),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
			LINE_AA); // ��ʾ����ʶ������

		putText(imageCorrect, std::to_string(bias_i), Point(100, 90),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
			LINE_AA); // ��ʾ����ʶ������
	}

	{
		putText(imageCorrect, "bias_d", Point(10, 120),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
			LINE_AA); // ��ʾ����ʶ������

		putText(imageCorrect, std::to_string(bias_d), Point(100, 120),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
			LINE_AA); // ��ʾ����ʶ������
	}

	{
		putText(imageCorrect, "midAdd", Point(10, 150),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
			LINE_AA); // ��ʾ����ʶ������

		putText(imageCorrect, std::to_string(midAdd), Point(100, 150),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
			LINE_AA); // ��ʾ����ʶ������
	}

	if (is_straight0) {
		putText(imageCorrect, "LStraight", Point(10, 170),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
			LINE_AA); // ��ʾ����ʶ������
	}
	else {
		putText(imageCorrect, "LStraight", Point(10, 170),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(255, 0, 0), 1,
			LINE_AA); // ��ʾ����ʶ������
	}
	if (is_straight1) {
		putText(imageCorrect, "RStraight", Point(150, 170),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
			LINE_AA); // ��ʾ����ʶ������
	}
	else {
		putText(imageCorrect, "RStraight", Point(150, 170),
			cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(255, 0, 0), 1,
			LINE_AA); // ��ʾ����ʶ������
	}
	static int ii = 1;
	std::string filename = "../frame/imageCorrect" + std::to_string(ii++) + ".jpg";
	imwrite(filename, imageCorrect);
	imshow("imageCorrect", imageCorrect);


}