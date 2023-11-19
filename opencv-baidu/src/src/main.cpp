#include "main.h"
#include "../code/headfile.h"

#include "../code/common.h"

#include "../code/imgproc.h"
#include "../code/debugger.h"
//用户访问图像数据直接访问这个指针变量就可以
//访问方式非常简单，可以直接使用下标的方式访问
//例如访问第10行 50列的点，mt9v03x_csi_image[10][50]就可以了
//uint8(*mt9v03x_csi_image)[MT9V03X_CSI_W];

image_t img_raw(NULL, COLSIMAGE, ROWSIMAGE);

int main() {

	return 0;
}