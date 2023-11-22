#ifndef __GARAGE_RECOGNITION_H__
#define __GARAGE_RECOGNITION_H__

enum garage_type_e {
    GARAGE_NONE = 0,                        // �ǳ���ģʽ
    GARAGE_OUT_LEFT, GARAGE_OUT_RIGHT,      // ���⣬������ת��45�㣬���������
    GARAGE_FOUND_LEFT, GARAGE_FOUND_RIGHT,  // ���ֳ��⣬��������+����L�ǵ�(δʹ��)
    GARAGE_IN_LEFT, GARAGE_IN_RIGHT,        // ���⣬���ֳ�����жϵڼ��Σ��Ӷ������Ƿ����
    GARAGE_PASS_LEFT, GARAGE_PASS_RIGHT,    // �����⣬���ֳ�����жϵڼ��Σ��Ӷ������Ƿ����
    GARAGE_STOP,                            // ������ϣ�ͣ��
    GARAGE_NUM,
};

#endif // !__GARAGE_RECOGNITION_H__