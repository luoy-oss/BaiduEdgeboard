#ifndef __TRACK_RECOGNITION_H__
#define __TRACK_RECOGNITION_H__
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

//Matͼ��תimage_t�ṹ��
void frameTOimg_raw(const cv::Mat& frame);
// ������ȡ&����
void process_image();
// �ǵ���ȡ&ɸѡ
void find_corners();

void show_line();

#endif // !__TRACK_RECOGNITION_H__
