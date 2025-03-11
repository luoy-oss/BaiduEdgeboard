#pragma once
#ifndef __COMMON_H__
#define __COMMON_H__

#define MIDX_MID 160.906

#define COLSIMAGE 320    // 图像的列数
#define ROWSIMAGE 240    // 图像的行数
//#define COLSIMAGEIPM 320 // IPM图像的列数
//#define ROWSIMAGEIPM 400 // IPM图像的行数
#define Servo_Center_Mid 4450      //舵机直行中值
#define Servo_Left_Max   5400      //舵机左转极限值
#define Servo_Right_Min  3500      //舵机右转极限值
// #define LABEL_BLOCK 0   // AI标签：障碍物
// #define LABEL_EVIL 1    // AI标签：恐怖分子
// #define LABEL_THIEF 2   // AI标签：盗贼
// #define LABEL_PATIENT 3 // AI标签：伤员
// #define LABEL_TUMBLE 4  // AI标签：跌倒
// #define LABEL_BRIDGE 5    // AI标签：坡道
// #define LABEL_BOMB 6      // AI标签：爆炸物
// #define LABEL_CONE 7      // AI标签：锥桶
// #define LABEL_CROSSWALK 8 // AI标签：斑马线


#define LABEL_CONE 0        // AI标签：锥桶
#define LABEL_BOMB 1        // AI标签：爆炸物
#define LABEL_BLOCK 2       // AI标签：障碍物
#define LABEL_BRIDGE 3      // AI标签：坡道
#define LABEL_CROSSWALK 4   // AI标签：斑马线
#define LABEL_PATIENT 5     // AI标签：伤员
#define LABEL_TUMBLE 6      // AI标签：跌倒
#define LABEL_EVIL 7        // AI标签：恐怖分子
#define LABEL_THIEF 8       // AI标签：盗贼
#define LABEL_CAR 9         // AI标签：车
#define LABEL_CARSAFE 10    // AI标签：安全车辆
#define LABEL_CARDANGER 11  // AI标签：危险车辆
#define LABEL_CARSUS 12     // AI标签：嫌疑车辆
#define LABEL_NUM 13


/*
cone
bomb
block
bridge
crosswalk
patient
fall
danger
thief
car
carsafe
cardanger
carsuspicion
*/
enum AItype{
    NONE = 0,
    BRIDGE,
    DANGER,
    PARKING,
    RACING,
    RESCUE
};

extern enum AItype ai_type;
// extern enum RescueType rescue_type;


#endif //! __COMMON_H__