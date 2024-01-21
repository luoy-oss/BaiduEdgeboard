#ifndef __CROSS_RECOGNITION_H__
#define __CROSS_RECOGNITION_H__


enum cross_type_e {
    CROSS_NONE = 0,     // ��ʮ��ģʽ
    CROSS_BEGIN,        // �ҵ���������L�ǵ�
    CROSS_IN,           // ����L�ǵ�ܽ���������ʮ���ڲ�(��ʱ�л�Զ�߿���)
    CROSS_HALF,
    CROSS_HALF_BEGIN,
    CROSS_HALF_LEFT,
    CROSS_HALF_RIGHT,
    CROSS_NUM,
};

extern enum cross_type_e cross_type;

extern const char* cross_type_name[CROSS_NUM];

void check_cross();

void check_Half();
void check_Half_left();
void check_Half_right();

void run_cross();

void draw_cross();

void cross_farline();
void cross_farline_L();
void cross_farline_R();

#endif // !__CROSS_RECOGNITION_H__