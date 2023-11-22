#include "circle_recognition.h"
#include "../src/main.h"
#include <vector>
#include <numeric>
#include <cmath>

using namespace std;
enum circle_type_e circle_type = CIRCLE_NONE;

//���㴮���շ�
const char* circle_type_name[CIRCLE_NUM] = {
        "CIRCLE_NONE",
        "CIRCLE_LEFT_BEGIN", "CIRCLE_RIGHT_BEGIN",
        "CIRCLE_LEFT_RUNNING", "CIRCLE_RIGHT_RUNNING",
        "CIRCLE_LEFT_IN", "CIRCLE_RIGHT_IN",
        "CIRCLE_LEFT_OUT", "CIRCLE_RIGHT_OUT",
        "CIRCLE_LEFT_END", "CIRCLE_RIGHT_END",
};

//// �����������ڷ�ֹһЩ�ظ������ȡ�
//int64_t circle_encoder;

int none_left_line = 0, none_right_line = 0;
int have_left_line = 0, have_right_line = 0;

void check_circle() {
    // ��Բ��ģʽ�£�����L�ǵ�, ���߳�ֱ��
    // ����Բ��  �յ����һ��  һ�߳�ֱ��
    if (circle_type == CIRCLE_NONE && Lpt0_found && !Lpt1_found && is_straight1) {
        circle_type = CIRCLE_LEFT_BEGIN;
    }
    if (circle_type == CIRCLE_NONE && !Lpt0_found && Lpt1_found && is_straight0) {
        circle_type = CIRCLE_RIGHT_BEGIN;
    }
}

void run_circle() {
    //****  2023.11.6
    // �󻷿�ʼ��Ѱ��ֱ������
    //**** track_type---Ѳ�߷�ʽ��־λ
    if (circle_type == CIRCLE_LEFT_BEGIN) {
        track_type = TRACK_RIGHT;

        //�ȶ����ߺ�����
        if (rpts0s_num < 0.2 / sample_dist) { none_left_line++; }//***�һ�����һ�����߶�ʧ�Ĺ���--�Ҳ���ߵ������
        if (rpts0s_num > 1.0 / sample_dist && none_left_line > 2) {//***ǰ����϶��߲��������³���
            have_left_line++;
            if (have_left_line > 1) {
                circle_type = CIRCLE_LEFT_IN;//***����
                none_left_line = 0;
                have_left_line = 0;
            }
        }
    }
    //�뻷��Ѱ��Բ����
    else if (circle_type == CIRCLE_LEFT_IN) {
        track_type = TRACK_LEFT;

        double stdevRight = 0; // ��Ե����
        if (rpts1s_num < ROWSIMAGE / 4) {
            stdevRight = 1000;
        }
        vector<int> v_slope;
        int step = 10; // size/10;
        if (rpts1s_num > 100) {
            for (int i = step; i < 100; i += step) {
                if (rpts1s[i][0] - rpts1s[i - step][0]) {
                    v_slope.push_back(
                        (rpts1s[i][0] - rpts1s[i - step][0]) * 100 /
                        (rpts1s[i][1] - rpts1s[i - step][1])
                    );
                }

            }
        }
        else {
            for (int i = step; i < rpts1s_num; i += step) {
                if (rpts1s[i][0] - rpts1s[i - step][0]) {
                    v_slope.push_back(
                        (rpts1s[i][0] - rpts1s[i - step][0]) * 100 /
                        (rpts1s[i][1] - rpts1s[i - step][1])
                    );
                }

            }
        }

        if (v_slope.size() > 1) {
            double sum = accumulate(begin(v_slope), end(v_slope), 0.0);
            double mean = sum / v_slope.size(); // ��ֵ
            double accum = 0.0;
            for_each(begin(v_slope), end(v_slope),
                [&](const double d) { accum += (d - mean) * (d - mean); });

            stdevRight = sqrt(accum / (v_slope.size() - 1)); // ����
        }
        else {
            stdevRight = 0;
        }
        if (stdevRight == 0) {
            none_right_line++;
        }

        /*if (none_right_line > 3) {
            cout <<"stdevRight::" << stdevRight << endl;
        }*/

        //����������1/4Բ   Ӧ����Ϊ����Ϊת���޹յ�
        //COUT1(rpts0s_num);
        //�����С��100
        if (rpts0s_num < 100/*0.2 / sample_dist*/ && none_right_line > 3 && stdevRight > 40/*||
            current_encoder - circle_encoder >= ENCODER_PER_METER * (3.14 * 1 / 2)*/) {
            circle_type = CIRCLE_LEFT_RUNNING;
            none_right_line = 0;
        }
        //***��������Ų������ߵ������ٽ��뻷��
    }
    //����Ѳ�ߣ�Ѱ��Բ����
    else if (circle_type == CIRCLE_LEFT_RUNNING) {
        track_type = TRACK_RIGHT;

        if (Lpt1_found) rpts1s_num = rptsc1_num = Lpt1_rpts1s_id;
        //�⻷�յ�(��L��)
        if (Lpt1_found && Lpt1_rpts1s_id < 0.4 / sample_dist) {
            circle_type = CIRCLE_LEFT_OUT;
        }
    }
    //������Ѱ��Բ
    else if (circle_type == CIRCLE_LEFT_OUT) {
        track_type = TRACK_LEFT;

        //����Ϊ��ֱ��
        if (is_straight1) {
            circle_type = CIRCLE_LEFT_END;
        }
    }
    //�߹�Բ����Ѱ����
    else if (circle_type == CIRCLE_LEFT_END) {
        track_type = TRACK_RIGHT;

        //�����ȶ�����
        if (rpts0s_num < 0.2 / sample_dist) { none_left_line++; }
        if (rpts0s_num > 1.0 / sample_dist && none_left_line > 3) {
            circle_type = CIRCLE_NONE;
            none_left_line = 0;
        }
    }
    //�һ����ƣ�ǰ��Ѱ��ֱ��
    else if (circle_type == CIRCLE_RIGHT_BEGIN) {
        track_type = TRACK_LEFT;

        //�ȶ����ߺ�����
        if (rpts1s_num < 0.2 / sample_dist) { none_right_line++; }
        if (rpts1s_num > 1.0 / sample_dist && none_right_line > 2) {
            have_right_line++;
            if (have_right_line > 1) {
                circle_type = CIRCLE_RIGHT_IN;
                none_right_line = 0;
                have_right_line = 0;
            }
        }
    }
    //���һ���Ѱ����Բ��
    else if (circle_type == CIRCLE_RIGHT_IN) {
        track_type = TRACK_RIGHT;

        double stdevLeft = 0; // ��Ե����
        if (rpts0s_num < ROWSIMAGE / 4) {
            stdevLeft = 1000;
        }
        vector<int> v_slope;
        int step = 10; // size/10;
        if (rpts0s_num > 100) {
            for (int i = step; i < 100; i += step) {
                if (rpts0s[i][0] - rpts0s[i - step][0]) {
                    v_slope.push_back(
                        (rpts0s[i][0] - rpts0s[i - step][0]) * 100 /
                        (rpts0s[i][1] - rpts0s[i - step][1])
                    );
                }

            }
        }
        else {
            for (int i = step; i < rpts0s_num; i += step) {
                if (rpts0s[i][0] - rpts0s[i - step][0]) {
                    v_slope.push_back(
                        (rpts0s[i][0] - rpts0s[i - step][0]) * 100 /
                        (rpts0s[i][1] - rpts0s[i - step][1])
                    );
                }

            }
        }
        
        if (v_slope.size() > 1) {
            double sum = accumulate(begin(v_slope), end(v_slope), 0.0);
            double mean = sum / v_slope.size(); // ��ֵ
            double accum = 0.0;
            for_each(begin(v_slope), end(v_slope),
                [&](const double d) { accum += (d - mean) * (d - mean); });

            stdevLeft = sqrt(accum / (v_slope.size() - 1)); // ����
        }
        else {
            stdevLeft = 0;
        }
        if (stdevLeft == 0) {
            none_left_line++;
        }
        /*if (none_left_line > 3) {
            cout <<"stdevLeft::" << stdevLeft << endl;
        }*/
        //����������1/4Բ   Ӧ����Ϊ����Ϊת���޹յ�
        //�ҵ���С��100
        if (rpts1s_num < 100/*0.2 / sample_dist*/ && none_left_line > 3 && stdevLeft > 40 /*||
            current_encoder - circle_encoder >= ENCODER_PER_METER * (3.14 * 1 / 2)*/) {
            circle_type = CIRCLE_RIGHT_RUNNING;
            none_left_line = 0;
        }

    }
    //����Ѳ�ߣ�Ѱ��Բ����
    else if (circle_type == CIRCLE_RIGHT_RUNNING) {
        track_type = TRACK_LEFT;

        //�⻷���ڹյ�,���ټӹյ�����о�(��L��)
        if (Lpt0_found) rpts0s_num = rptsc0_num = Lpt0_rpts0s_id;
        if (Lpt0_found && Lpt0_rpts0s_id < 0.4 / sample_dist) {
            circle_type = CIRCLE_RIGHT_OUT;
        }
    }
    //������Ѱ��Բ
    else if (circle_type == CIRCLE_RIGHT_OUT) {
        track_type = TRACK_RIGHT;

        //�󳤶ȼ���б�Ƕ�  Ӧ�����������ҵ���Ϊֱ��
        //if((rpts1s_num >100 && !Lpt1_found))  {have_right_line++;}
        if (is_straight0) {
            circle_type = CIRCLE_RIGHT_END;
        }
    }
    //�߹�Բ����Ѱ����
    else if (circle_type == CIRCLE_RIGHT_END) {
        track_type = TRACK_LEFT;

        //�����ȶ�����
        if (rpts1s_num < 0.2 / sample_dist) { none_right_line++; }
        if (rpts1s_num > 1.0 / sample_dist && none_right_line > 2) {
            circle_type = CIRCLE_NONE;
            none_right_line = 0;
        }
    }
}

// ����Բ��ģʽ�µĵ���ͼ��
void draw_circle() {

}

