#ifndef __TRACK_RECOGNITION_H__
#define __TRACK_RECOGNITION_H__
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

//Mat图像转image_t结构体
void frameTOimg_raw(const cv::Mat& frame);
// 边线提取
void get_normal_line();
// 边线处理
void line_process();
// 角点提取&筛选
void find_corners();
// 边线切换
void track_switch();

void show_line();

#endif // !__TRACK_RECOGNITION_H__
