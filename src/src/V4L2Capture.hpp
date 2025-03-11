#ifndef __MY_V4L2CAPTURE_H__
#define __MY_V4L2CAPTURE_H__

#include <unistd.h>
#include <error.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <pthread.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>
#include <fstream>
#include "../code/json.hpp"
#define CLEAR(x) memset(&(x), 0, sizeof(x))

using namespace std;

class V4L2Capture {
public:
    int WHITE_BALANCE_TEMPERATURE = 4000;		//白平衡
    int BRIGHTNESS = -30;			//亮度
    int CONTRAST = 32;				//对比度
    int SATURATION = 32;			//饱和度
    int HUE = 0;					//色度
    int SHARPNESS = 0;				//锐度
    int BACKLIGHT_COMPENSATION = 1;	//背光补偿
    int GAMMA = 72;					//伽马
    int EXPOSURE_ABSOLUTE = 0;		//曝光绝对值

    struct CameraParams {
        int WHITE_BALANCE_TEMPERATURE = 4000;		//白平衡
        int BRIGHTNESS = -30;			//亮度
        int CONTRAST = 32;				//对比度
        int SATURATION = 32;			//饱和度
        int HUE = 0;					//色度
        int SHARPNESS = 0;				//锐度
        int BACKLIGHT_COMPENSATION = 1;	//背光补偿
        int GAMMA = 72;					//伽马
        int EXPOSURE_ABSOLUTE = 0;		//曝光绝对值
        
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            CameraParams, 
            WHITE_BALANCE_TEMPERATURE,
            BRIGHTNESS,
            CONTRAST,
            SATURATION,
            HUE,
            SHARPNESS,
            BACKLIGHT_COMPENSATION,
            GAMMA,
            EXPOSURE_ABSOLUTE); // 添加构造函数
    };
	V4L2Capture(char *devName, int width, int height);
	virtual ~V4L2Capture(); 

    void __init__() {
        CameraParams params;
        string jsonPath = "../src/config/camera_config.json";
        std::ifstream config_is(jsonPath);
        if (!config_is.good()) {
            std::cout << "Error: Params file path:[" << jsonPath << "] not find .\n";
            exit(-1);
        }

        nlohmann::json js_value;
        config_is >> js_value;

        try {
            params = js_value.get<CameraParams>();
        } catch (const nlohmann::detail::exception &e) {
            std::cerr << "Json params Parse failed :" << e.what() << '\n';
            exit(-1);
        }
        std::cout <<"--- 白平衡: \t" 	<< params.WHITE_BALANCE_TEMPERATURE << std::endl
            <<"--- 亮度: \t" 		<< params.BRIGHTNESS << std::endl
            <<"--- 对比度: \t" 	    << params.CONTRAST << std::endl
            <<"--- 饱和度: \t" 	    << params.SATURATION << std::endl
            <<"--- 色度: \t" 		<< params.HUE << std::endl
            <<"--- 锐度: \t" 		<< params.SHARPNESS << std::endl
            <<"--- 曝光补偿: \t" 	<< params.BACKLIGHT_COMPENSATION << std::endl
            <<"--- 伽马: \t" 	    << params.GAMMA << std::endl
            <<"--- 曝光绝对值: \t"  << params.EXPOSURE_ABSOLUTE << std::endl;

        this->WHITE_BALANCE_TEMPERATURE = params.WHITE_BALANCE_TEMPERATURE;
        this->BRIGHTNESS = params.BRIGHTNESS;
        this->CONTRAST = params.CONTRAST;
        this->SATURATION = params.SATURATION;
        this->HUE = params.HUE;
        this->SHARPNESS = params.SHARPNESS;
        this->BACKLIGHT_COMPENSATION = params.BACKLIGHT_COMPENSATION;
        this->GAMMA = params.GAMMA;
        this->EXPOSURE_ABSOLUTE = params.EXPOSURE_ABSOLUTE;
    }

	int openDevice();
	int closeDevice();
	int initDevice();
	int startCapture();
	int stopCapture();
	int freeBuffers();
	int getFrame(void **,size_t *);
	int backFrame();
	static void test();

private:
	int initBuffers() {
        int ret;
        /* 使用IOCTL命令VIDIOC_REQBUFS，申请帧缓冲*/
        struct v4l2_requestbuffers req;
        CLEAR(req);
        req.count = 4;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        ret = ioctl(fd_cam, VIDIOC_REQBUFS, &req);
        if (ret < 0) {
            perror("Request frame buffers failed");
        }
        if (req.count < 2) {
            perror("Request frame buffers while insufficient buffer memory");
        }
        buffers = (struct cam_buffer*) calloc(req.count, sizeof(*buffers));
        if (!buffers) {
            perror("Out of memory");
        }
        for (n_buffers = 0; n_buffers < req.count; n_buffers++) {
            struct v4l2_buffer buf;
            CLEAR(buf);
            // 查询序号为n_buffers 的缓冲区，得到其起始物理地址和大小
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = n_buffers;
            ret = ioctl(fd_cam, VIDIOC_QUERYBUF, &buf);
            if (ret < 0) {
                printf("VIDIOC_QUERYBUF %d failed\n", n_buffers);
                return -1;
            }
            buffers[n_buffers].length = buf.length;
            //printf("buf.length= %d\n",buf.length);
            // 映射内存
            buffers[n_buffers].start = mmap(
                    NULL, // start anywhere
                    buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_cam,
                    buf.m.offset);
            if (MAP_FAILED == buffers[n_buffers].start) {
                printf("mmap buffer%d failed\n", n_buffers);
                return -1;
            }
        }
        return 0;
    }

	struct cam_buffer
	{
		void* start;
		unsigned int length;
	};
	char *devName;
	int capW;
	int capH;
	int fd_cam;
	cam_buffer *buffers;
	unsigned int n_buffers;
	int frameIndex;
};


#endif