#ifndef CAMERA_PARAM_H
#define CAMERA_PARAM_H

#include <stdbool.h>

// ��任(����->ԭͼ)
bool map_inv(float pt0[2], int pt1[2]);

extern double H[3][3];
extern double mapx[240][320];
extern double mapy[240][320];

#endif // CAMERA_PARAM_H
