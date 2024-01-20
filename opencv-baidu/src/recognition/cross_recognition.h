#ifndef __CROSS_RECOGNITION_H__
#define __CROSS_RECOGNITION_H__


enum cross_type_e {
    CROSS_NONE = 0,     // ��ʮ��ģʽ
    CROSS_BEGIN,        // �ҵ���������L�ǵ�
    CROSS_IN,           // ����L�ǵ�ܽ���������ʮ���ڲ�(��ʱ�л�Զ�߿���)
    CROSS_BEGIN_HALF_LEFT,      // �ҵ���������L�ǵ�(����)
    CROSS_IN_HALF_LEFT,         // ����L�ǵ�ܽ���������ʮ���ڲ�(��ʱ�л�Զ�߿���)(����)
    CROSS_BEGIN_HALF_RIGHT,     // �ҵ���������L�ǵ�(�Ұ��)
    CROSS_IN_HALF_RIGHT,        // ����L�ǵ�ܽ���������ʮ���ڲ�(��ʱ�л�Զ�߿���)(�Ұ��)
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