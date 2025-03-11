#ifndef __FUZZY_PID_H__
#define __FUZZY_PID_H__

class FuzzyPID{
public:
    FuzzyPID();
    ~FuzzyPID();
    void _init_(float _kp);
    float _P(float E, float EC);
    float _D(float E, float EC);
/*
#define NB   3.6
#define NM   3.53
#define NS   3.52
#define ZO   3.48
#define PS   3.50
#define PM   3.53
#define PB   3.55



#define NB1   7.80
#define NM1   7.74
#define NS1   7.71
#define ZO1   7.6
#define PS1   7.7
#define PM1   7.77
#define PB1   7.78
*/
    int P_NB = -3, P_NM = -2, P_NS = -1, P_ZO = 0, P_PS = 1, P_PM = 2, P_PB = 3; //KP 论域隶属值
    int D_NB = -3, D_NM = -2, D_NS = -1, D_ZO = 0, D_PS = 1, D_PM = 2, D_PB = 3; //KD 论域隶属值

    //    //先标定误差范围         NB，NM，NS，ZO，PS，PM，PB
    float error[7] = { -12, -6, -3.5, 0, 3.5, 6, 12};
    //    //标定误差变化率范围
    float error_c[7] = { -3, -2, -0.5, 0, 0.5, 2, 3};

    int  Kp_rule_list[7][7] = { {P_PB, P_PB, P_PB, P_PB, P_PM, P_PS, P_ZO},
                                {P_PB, P_PB, P_PB, P_PM, P_PM, P_ZO, P_ZO},
                                {P_PB, P_PM, P_PM, P_PS, P_ZO, P_NS, P_NM},
                                {P_PM, P_PM, P_PS, P_ZO, P_NS, P_NM, P_NM},
                                {P_PS, P_PS, P_ZO, P_NM, P_NM, P_NM, P_NB},
                                {P_ZO, P_ZO, P_ZO, P_NM, P_NB, P_NB, P_NB},
                                {P_ZO, P_NS, P_NB, P_NB, P_NB, P_NB, P_NB} };

    int  Kd_rule_list[7][7] = { {D_PB, D_PB, D_PB, D_PB, D_PM, D_PS, D_ZO},
                                {D_PB, D_PB, D_PB, D_PM, D_PM, D_ZO, D_ZO},
                                {D_PB, D_PM, D_PM, D_PS, D_ZO, D_NS, D_NM},
                                {D_PM, D_PM, D_PS, D_ZO, D_NS, D_NM, D_NM},
                                {D_PS, D_PS, D_ZO, D_NM, D_NM, D_NM, D_NB},
                                {D_ZO, D_ZO, D_ZO, D_NM, D_NB, D_NB, D_NB},
                                {D_ZO, D_NS, D_NB, D_NB, D_NB, D_NB, D_NB} };


};

#endif