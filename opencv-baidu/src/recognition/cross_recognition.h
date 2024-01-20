#ifndef __CROSS_RECOGNITION_H__
#define __CROSS_RECOGNITION_H__


enum cross_type_e {
    CROSS_NONE = 0,     // 非十字模式
    CROSS_BEGIN,        // 找到左右两个L角点
    CROSS_IN,           // 两个L角点很近，即进入十字内部(此时切换远线控制)
    CROSS_BEGIN_HALF_LEFT,      // 找到左右两个L角点(左半边)
    CROSS_IN_HALF_LEFT,         // 两个L角点很近，即进入十字内部(此时切换远线控制)(左半边)
    CROSS_BEGIN_HALF_RIGHT,     // 找到左右两个L角点(右半边)
    CROSS_IN_HALF_RIGHT,        // 两个L角点很近，即进入十字内部(此时切换远线控制)(右半边)
    CROSS_NUM,
};

extern enum cross_type_e cross_type;

extern const char* cross_type_name[CROSS_NUM];

void half_check();

void run_cross_half();

void check_cross();

void run_cross();

void draw_cross();

void cross_farline();

#endif // !__CROSS_RECOGNITION_H__