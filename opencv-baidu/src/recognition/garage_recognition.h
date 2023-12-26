#ifndef __GARAGE_RECOGNITION_H__
#define __GARAGE_RECOGNITION_H__

enum garage_type_e {
    GARAGE_NONE = 0,// �ǳ���ģʽ
    GARAGE_OUT,     // ���⣬������ת��45�㣬���������
    GARAGE_FOUND,   // ���ֳ��⣬��������+����L�ǵ�(δʹ��)
    GARAGE_IN,      // ���⣬���ֳ�����жϵڼ��Σ��Ӷ������Ƿ����
    GARAGE_PASS,    // �����⣬���ֳ�����жϵڼ��Σ��Ӷ������Ƿ����
    GARAGE_STOP,    // ������ϣ�ͣ��
    GARAGE_NUM,
};

extern enum garage_type_e garage_type;

extern const char* garage_type_name[GARAGE_NUM];

void check_garage();

void run_garage();

void draw_garage();

#endif // !__GARAGE_RECOGNITION_H__