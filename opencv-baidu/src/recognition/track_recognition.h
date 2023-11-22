#ifndef __TRACK_RECOGNITION_H__
#define __TRACK_RECOGNITION_H__
#include "../code/headfile.h"
#include "../src/main.h"
#include "../code/camera_param.h"
#include "../code/imgproc.h"

void frameTOimg_raw(const cv::Mat& frame);
void process_image();


#endif // !__TRACK_RECOGNITION_H__
