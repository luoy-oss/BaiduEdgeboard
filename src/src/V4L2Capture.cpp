#include "./V4L2Capture.hpp"
using namespace std;


V4L2Capture::V4L2Capture(char *devName, int width, int height) {
	// TODO Auto-generated constructor stub
	this->devName = devName;
	this->fd_cam = -1;
	this->buffers = NULL;
	this->n_buffers = 0;
	this->frameIndex = -1;
	this->capW=width;
	this->capH=height;
}

V4L2Capture::~V4L2Capture() {
	// TODO Auto-generated destructor stub
}

int V4L2Capture::openDevice() {
	/*设备的打开*/
	printf("video dev : %s\n", devName);
	fd_cam = open(devName, O_RDWR);
	if (fd_cam < 0) {
		perror("Can't open video device");
	}
	return 0;
}

int V4L2Capture::closeDevice() {
	if (fd_cam > 0) {
		int ret = 0;
		if ((ret = close(fd_cam)) < 0) {
			perror("Can't close video device");
		}
		return 0;
	} else {
		return -1;
	}
}

int V4L2Capture::initDevice() {

	V4L2Capture::__init__();	//摄像头参数初始化

	int ret;
	struct v4l2_capability cam_cap;		//显示设备信息
	struct v4l2_cropcap cam_cropcap;	//设置摄像头的捕捉能力
	struct v4l2_fmtdesc cam_fmtdesc;	//查询所有支持的格式：VIDIOC_ENUM_FMT
	struct v4l2_crop cam_crop;			//图像的缩放
	struct v4l2_format cam_format;		//设置摄像头的视频制式、帧格式等
	struct v4l2_control cam_ctrl;

	/* 使用IOCTL命令VIDIOC_QUERYCAP，获取摄像头的基本信息*/
	ret = ioctl(fd_cam, VIDIOC_QUERYCAP, &cam_cap);
	if (ret < 0) {
		perror("Can't get device information: VIDIOCGCAP");
	}
	printf(
			"Driver Name:%s\nCard Name:%s\nBus info:%s\nDriver Version:%u.%u.%u\n",
			cam_cap.driver, cam_cap.card, cam_cap.bus_info,
			(cam_cap.version >> 16) & 0XFF, (cam_cap.version >> 8) & 0XFF,
			cam_cap.version & 0XFF);

	/* 使用IOCTL命令VIDIOC_ENUM_FMT，获取摄像头所有支持的格式*/
	cam_fmtdesc.index = 0;
	cam_fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	printf("Support format:\n");
	while (ioctl(fd_cam, VIDIOC_ENUM_FMT, &cam_fmtdesc) != -1) {
		printf("\t%d.%s\n", cam_fmtdesc.index + 1, cam_fmtdesc.description);
		cam_fmtdesc.index++;
	}

	/* 使用IOCTL命令VIDIOC_CROPCAP，获取摄像头的捕捉能力*/
	cam_cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (0 == ioctl(fd_cam, VIDIOC_CROPCAP, &cam_cropcap)) {
		printf("Default rec:\n\tleft:%d\n\ttop:%d\n\twidth:%d\n\theight:%d\n",
				cam_cropcap.defrect.left, cam_cropcap.defrect.top,
				cam_cropcap.defrect.width, cam_cropcap.defrect.height);
		/* 使用IOCTL命令VIDIOC_S_CROP，获取摄像头的窗口取景参数*/
		cam_crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cam_crop.c = cam_cropcap.defrect;		//默认取景窗口大小
		if (-1 == ioctl(fd_cam, VIDIOC_S_CROP, &cam_crop)) {
			//printf("Can't set crop para\n");
		}
	} else {
		printf("Can't set cropcap para\n");
	}

	/* 使用IOCTL命令VIDIOC_S_FMT，设置摄像头帧信息*/
	cam_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	cam_format.fmt.pix.width = capW;
	cam_format.fmt.pix.height = capH;
	cam_format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;		//要和摄像头支持的类型对应
	cam_format.fmt.pix.field = V4L2_FIELD_INTERLACED;
	ret = ioctl(fd_cam, VIDIOC_S_FMT, &cam_format);
	if (ret < 0) {
		perror("Can't set frame information");
	}
	/* 使用IOCTL命令VIDIOC_G_FMT，获取摄像头帧信息*/
	cam_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ret = ioctl(fd_cam, VIDIOC_G_FMT, &cam_format);
	if (ret < 0) {
		perror("Can't get frame information");
	}
	printf("Current data format information:\n\twidth:%d\n\theight:%d\n",
			cam_format.fmt.pix.width, cam_format.fmt.pix.height);
	ret = initBuffers();
	if (ret < 0) {
		perror("Buffers init error");
		//exit(-1);
	}

    printf("【**********************设置手动白平衡：******************************】\n");
    cam_ctrl.id = V4L2_CID_AUTO_WHITE_BALANCE;
    cam_ctrl.value = V4L2_WHITE_BALANCE_MANUAL;
    if(ioctl(fd_cam,VIDIOC_G_CTRL,&cam_ctrl)==-1)
    {
        perror("ioctl");
        exit(EXIT_FAILURE);
    }
    printf("\n");    
 
    /*************设置白平衡色温****************************/
    printf("【****************设置白平衡色温********************】\n");
    cam_ctrl.id = V4L2_CID_WHITE_BALANCE_TEMPERATURE;
    cam_ctrl.value = WHITE_BALANCE_TEMPERATURE;
     if(ioctl(fd_cam,VIDIOC_S_CTRL,&cam_ctrl)==-1)
    {
        perror("ioctl");
        exit(EXIT_FAILURE);
    }
    printf("\n");
 
    printf("【***************设置亮度***************************】\n");
    cam_ctrl.id= V4L2_CID_BRIGHTNESS;
    cam_ctrl.value = BRIGHTNESS;
    if(ioctl(fd_cam,VIDIOC_S_CTRL,&cam_ctrl)==-1)
    {
        perror("ioctl");
        exit(EXIT_FAILURE);
    }
    printf("\n");
 
    printf("【***************设置对比度**************************】\n");
    cam_ctrl.id = V4L2_CID_CONTRAST;
    cam_ctrl.value= CONTRAST;
    if(ioctl(fd_cam,VIDIOC_S_CTRL,&cam_ctrl)==-1)
    {
        perror("ioctl");
        exit(EXIT_FAILURE);
    }
    sleep(1);
    printf("\n");
 
    printf("【***************设置饱和度**************************】\n");
    cam_ctrl.id = V4L2_CID_SATURATION;
    cam_ctrl.value= SATURATION;
    if(ioctl(fd_cam,VIDIOC_S_CTRL,&cam_ctrl)==-1)
    {
        perror("ioctl");
        exit(EXIT_FAILURE);
    }
    printf("\n");
 
    printf("【********************设置色度**********************】\n");
    cam_ctrl.id = V4L2_CID_HUE;
    cam_ctrl.value = HUE;
    if(ioctl(fd_cam,VIDIOC_S_CTRL,&cam_ctrl)==-1)
    {
        perror("ioctl");
        exit(EXIT_FAILURE);
    }
    printf("\n");
 
    printf("【********************设置锐度**********************】\n");
    cam_ctrl.id = V4L2_CID_SHARPNESS;
    cam_ctrl.value = SHARPNESS;
    if(ioctl(fd_cam,VIDIOC_S_CTRL,&cam_ctrl)==-1)
    {
        perror("ioctl");
        exit(EXIT_FAILURE);
    }
    printf("\n");
 
    
    printf("【*******************设置背光补偿******************】\n");
    cam_ctrl.id = V4L2_CID_BACKLIGHT_COMPENSATION;
    cam_ctrl.value = BACKLIGHT_COMPENSATION;
    if(ioctl(fd_cam,VIDIOC_S_CTRL,&cam_ctrl)==-1)
    {
        perror("ioctl");
        exit(EXIT_FAILURE);
    }
    printf("\n");
 
    printf("【********************设置伽玛值**********************】\n");
    cam_ctrl.id = V4L2_CID_GAMMA;
    cam_ctrl.value = GAMMA;
    if(ioctl(fd_cam,VIDIOC_S_CTRL,&cam_ctrl)==-1) {
        perror("ioctl");
        exit(EXIT_FAILURE);
    }
    printf("\n");

    printf("【********************设置手动曝光**********************】\n");
	cam_ctrl.id = V4L2_CID_EXPOSURE_AUTO; 
	cam_ctrl.value = V4L2_EXPOSURE_MANUAL;  //手动曝光模式 
	if (ioctl(fd_cam, VIDIOC_S_CTRL, &cam_ctrl) == -1)  {
        perror("ioctl");
        exit(EXIT_FAILURE);
	}
    printf("\n");

    printf("【********************设置曝光绝对值**********************】\n");
	//设置曝光绝对值
	cam_ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
	cam_ctrl.value = EXPOSURE_ABSOLUTE;  //单位100us
	if (ioctl(fd_cam, VIDIOC_S_CTRL, &cam_ctrl) == -1) {
        perror("ioctl");
        exit(EXIT_FAILURE);
	}


	return 0;
}


int V4L2Capture::startCapture() {
	unsigned int i;
	for (i = 0; i < n_buffers; i++) {
		struct v4l2_buffer buf;
		CLEAR(buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		if (-1 == ioctl(fd_cam, VIDIOC_QBUF, &buf)) {
			printf("VIDIOC_QBUF buffer%d failed\n", i);
			return -1;
		}
	}
	enum v4l2_buf_type type;
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == ioctl(fd_cam, VIDIOC_STREAMON, &type)) {
		printf("VIDIOC_STREAMON error");
		return -1;
	}
	return 0;
}

int V4L2Capture::stopCapture() {
	enum v4l2_buf_type type;
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == ioctl(fd_cam, VIDIOC_STREAMOFF, &type)) {
		printf("VIDIOC_STREAMOFF error\n");
		return -1;
	}
	return 0;
}

int V4L2Capture::freeBuffers() {
	unsigned int i;
	for (i = 0; i < n_buffers; ++i) {
		if (-1 == munmap(buffers[i].start, buffers[i].length)) {
			printf("munmap buffer%d failed\n", i);
			return -1;
		}
	}
	free(buffers);
	return 0;
}

int V4L2Capture::getFrame(void **frame_buf, size_t* len) {
	struct v4l2_buffer queue_buf;
	CLEAR(queue_buf);
	queue_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	queue_buf.memory = V4L2_MEMORY_MMAP;
	if (-1 == ioctl(fd_cam, VIDIOC_DQBUF, &queue_buf)) {
		printf("VIDIOC_DQBUF error\n");
		return -1;
	}
	*frame_buf = buffers[queue_buf.index].start;
	*len = buffers[queue_buf.index].length;
	frameIndex = queue_buf.index;
	return 0;
}

int V4L2Capture::backFrame() {
	if (frameIndex != -1) {
		struct v4l2_buffer queue_buf;
		CLEAR(queue_buf);
		queue_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		queue_buf.memory = V4L2_MEMORY_MMAP;
		queue_buf.index = frameIndex;
		if (-1 == ioctl(fd_cam, VIDIOC_QBUF, &queue_buf)) {
			printf("VIDIOC_QBUF error\n");
			return -1;
		}
		return 0;
	}
	return -1;
}

void V4L2Capture::test() {
	unsigned char *yuv422frame = NULL;
	unsigned long yuvframeSize = 0;

	string videoDev="/dev/video0";
	V4L2Capture *vcap = new V4L2Capture(const_cast<char*>(videoDev.c_str()),
			640, 480);
	vcap->openDevice();
	vcap->initDevice();
	vcap->startCapture();
	vcap->getFrame((void **) &yuv422frame, (size_t *)&yuvframeSize);

	vcap->backFrame();
	vcap->freeBuffers();
	vcap->closeDevice();
}
