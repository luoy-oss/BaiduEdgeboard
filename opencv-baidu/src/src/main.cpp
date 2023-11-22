#include "main.h"
#include "../code/headfile.h"

#include "../code/common.h"

#include "../code/debugger.h"
#include "../code/imgproc.h"
#include "../code/camera_param.h"

#include "../recognition/track_recognition.h"
using namespace cv;

//用户访问图像数据直接访问这个指针变量就可以
//访问方式非常简单，可以直接使用下标的方式访问
//例如访问第10行 50列的点，mt9v03x_csi_image[10][50]就可以了
//uint8(*mt9v03x_csi_image)[MT9V03X_CSI_W];
uint8_t img_thres_data[ROWSIMAGE][COLSIMAGE];

//image_t img_raw(NULL, COLSIMAGE, ROWSIMAGE);
image_t img_thres((uint8_t*)img_thres_data, COLSIMAGE, ROWSIMAGE);

int main() {


	Mat frame;
	VideoCapture cap(0);
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
		cap >> frame;
		cvtColor(frame, frame, COLOR_BGR2GRAY);
		threshold(frame, frame, 0, 255, THRESH_OTSU);
		frameTOimg_raw(frame);
		process_image();
		imshow("frame", frame);
		waitKey(1);
	}
	return 0;
}