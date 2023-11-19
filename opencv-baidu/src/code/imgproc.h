#ifndef __IMGPROC_H__
#define __IMGPROC_H__

#include "common.h"
#include <stdint.h>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署

typedef struct image {
    uint8_t* data;
    uint32_t width;
    uint32_t height;
    uint32_t step;
    
    image(uint8_t* data, const uint32_t width, const uint32_t height) {
        this->data = data;
        this->width = width;
        this->height = height;
        this->step = width;
    }
    void MatClone(const cv::Mat frame) {
        uint8_t* temp = (uint8_t*)malloc(sizeof(uint8_t) * ROWSIMAGE * COLSIMAGE);

    }
    
} image_t;

typedef struct fimage {
    float* data;
    uint32_t width;
    uint32_t height;
    uint32_t step;
} fimage_t;

#define AT_IMAGE(img, x, y)          ((img)->data[(y)*(img)->step+(x)])
#define AT_IMAGE_CLIP(img, x, y)     AT_IMAGE(img, clip(x, 0, (img)->width-1), clip(y, 0, (img)->height-1))

#define DEF_IMAGE(ptr, w, h)         {.data=ptr, .width=w, .height=h, .step=w}
#define ROI_IMAGE(img, x1, y1, w, h) {.data=&AT_IMAGE(img, x1, y1), .width=w, .height=h, .step=img.width}


// 左手巡线同时自适应二值化，从(x,y)开始向上沿白边线走
void findline_lefthand_adaptive(image_t* img, int block_size, int clip_value, int x, int y, int pts[][2], int* num);

// 右手巡线同时自适应二值化，从(x,y)开始向上沿白边线走
void findline_righthand_adaptive(image_t* img, int block_size, int clip_value, int x, int y, int pts[][2], int* num);


// 点集三角滤波
void blur_points(float pts_in[][2], int num, float pts_out[][2], int kernel);

// 点集等距采样
void resample_points(float pts_in[][2], int num1, float pts_out[][2], int* num2, float dist);

// 点集局部角度变化率
void local_angle_points(float pts_in[][2], int num, float angle_out[], int dist);

// 角度变化率非极大抑制
void nms_angle(float angle_in[], int num, float angle_out[], int kernel);

// 左边线跟踪中线
void track_leftline(float pts_in[][2], int num, float pts_out[][2], int approx_num, float dist);

// 右边线跟踪中线
void track_rightline(float pts_in[][2], int num, float pts_out[][2], int approx_num, float dist);



#endif //! __IMGPROC_H__