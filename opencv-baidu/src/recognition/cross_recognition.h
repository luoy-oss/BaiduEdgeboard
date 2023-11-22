#ifndef __CROSS_RECOGNITION_H__
#define __CROSS_RECOGNITION_H__


enum cross_type_e {
    CROSS_NONE = 0,     // ��ʮ��ģʽ
    CROSS_BEGIN,        // �ҵ���������L�ǵ�
    CROSS_IN,           // ����L�ǵ�ܽ���������ʮ���ڲ�(��ʱ�л�Զ�߿���)
    CROSS_NUM,
};

extern enum cross_type_e cross_type;

extern const char* cross_type_name[CROSS_NUM];

void check_cross();

void run_cross();

void draw_cross();

void cross_farline();

#endif // !__CROSS_RECOGNITION_H__