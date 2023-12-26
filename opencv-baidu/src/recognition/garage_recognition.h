#ifndef __GARAGE_RECOGNITION_H__
#define __GARAGE_RECOGNITION_H__

enum garage_type_e {
    GARAGE_NONE = 0,// 非车库模式
    GARAGE_OUT,     // 出库，陀螺仪转过45°，即出库完毕
    GARAGE_FOUND,   // 发现车库，即斑马线+单侧L角点(未使用)
    GARAGE_IN,      // 进库，发现车库后判断第几次，从而决定是否进库
    GARAGE_PASS,    // 不进库，发现车库后判断第几次，从而决定是否进库
    GARAGE_STOP,    // 进库完毕，停车
    GARAGE_NUM,
};

extern enum garage_type_e garage_type;

extern const char* garage_type_name[GARAGE_NUM];

void check_garage();

void run_garage();

void draw_garage();

#endif // !__GARAGE_RECOGNITION_H__