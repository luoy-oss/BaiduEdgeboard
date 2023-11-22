#ifndef __TRACK_RECOGNITION_H__
#define __TRACK_RECOGNITION_H__
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

//Mat图像转image_t结构体
void frameTOimg_raw(const cv::Mat& frame);
// 边线提取&处理
void process_image();
// 角点提取&筛选
void find_corners();

void show_line();

#endif // !__TRACK_RECOGNITION_H__
