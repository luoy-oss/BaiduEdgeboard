#include "V4L2Capture.cpp"
using namespace std;

void VideoPlayer() {
	unsigned char *yuv422frame = NULL;
	unsigned long yuvframeSize = 0;

	string videoDev = "/dev/video0";
	V4L2Capture *vcap = new V4L2Capture(const_cast<char*>(videoDev.c_str()), 320, 240);
	vcap->openDevice();
	vcap->initDevice();
	vcap->startCapture();

	cvNamedWindow("Capture",CV_WINDOW_AUTOSIZE);
	IplImage* img;
	CvMat cvmat;
	double t;
	while(1){
		t = (double)cvGetTickCount();
		vcap->getFrame((void **) &yuv422frame, (size_t *)&yuvframeSize);
		cvmat = cvMat(240, 320, CV_8UC3,(void*)yuv422frame);		//CV_8UC3


		//解码
		img = cvDecodeImage(&cvmat,1);
		if(!img){
			printf("DecodeImage error!\n");
		}


        cv::Mat m = cv::cvarrToMat(img);  
        cv::imshow("m",m);
		// cvShowImage("Capture",img);
		cvReleaseImage(&img);

		vcap->backFrame();
		if((cvWaitKey(1)&255) == 27){
			exit(0);
		}
		t = (double)cvGetTickCount() - t;
		printf("Used time is %g ms\n",( t / (cvGetTickFrequency()*1000)));
	}		
	vcap->stopCapture();
	vcap->freeBuffers();
	vcap->closeDevice();

}

int main() {
	VideoPlayer();
	return 0;
}