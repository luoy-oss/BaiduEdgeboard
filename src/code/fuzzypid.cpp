#include "fuzzypid.h"


//传入参数误差和误差变化率
float FuzzyPID::_P(float E, float EC) {
    int Pn = 0, Dn = 0;
    //    //输出量即根据隶属度可能取得的P
    float UP = 0;
    // //    //模糊规则表的制定：
    // float Kp_rule_list[7][7] = {
    //     {PB, PB, PB, PB, PM, PS, ZO},
    //     {PB, PB, PB, PM, PM, ZO, ZO},
    //     {PB, PM, PM, PS, ZO, NS, NM},
    //     {PM, PM, PS, ZO, NS, NM, NM},
    //     {PS, PS, ZO, NM, NM, NM, NB},
    //     {ZO, ZO, ZO, NM, NB, NB, NB},
    //     {ZO, NS, NB, NB, NB, NB, NB}
    // };


    //    //E的隶属度求法
    //    ///
    //    ///
    //
    float E_belong_front = 0; float E_belong_behind = 0;
    if (E > error[0] && E < error[6]) {
        if (E < error[1]) {
            Pn = 0;
            E_belong_front = (error[1] - E) / (error[1] - error[0]);
        }
        else if (E < error[2]) {
            Pn = 1;
            E_belong_front = (error[2] - E) / (error[2] - error[1]);
        }
        else if (E < error[3]) {
            Pn = 2;
            E_belong_front = (error[3] - E) / (error[3] - error[2]);
        }
        else if (E < error[4]) {
            Pn = 3;
            E_belong_front = (error[4] - E) / (error[4] - error[3]);
        }
        else if (E < error[5]) {
            Pn = 4;
            E_belong_front = (error[5] - E) / (error[5] - error[4]);
        }
        else if (E < error[6]) {
            Pn = 5;
            E_belong_front = (error[6] - E) / (error[6] - error[5]);
        }
        E_belong_behind = 1 - E_belong_front;
    }


    //EC的隶属度求法
    float EC_belong_front = 0; float EC_belong_behind = 0;
    if (EC > error_c[0] && EC < error_c[6]) {
        if (EC < error_c[1]) {
            Dn = 0; EC_belong_front = (error_c[1] - EC) / (error_c[1] - error_c[0]);
        }
        else if (EC < error_c[2]) {
            Dn = 1; EC_belong_front = (error_c[2] - EC) / (error_c[2] - error_c[1]);
        }
        else if (EC < error_c[3]) {
            Dn = 2;
            EC_belong_front = (error_c[3] - EC) / (error_c[3] - error_c[2]);
        }
        else if (EC < error_c[4]) {
            Dn = 3;
            EC_belong_front = (error_c[4] - EC) / (error_c[4] - error_c[3]);
        }
        else if (EC < error_c[5]) {
            Dn = 4;
            EC_belong_front = (error_c[5] - EC) / (error_c[5] - error_c[4]);
        }
        else if (EC < error_c[6]) {
            Dn = 5;
            EC_belong_front = (error_c[6] - EC) / (error_c[6] - error_c[5]);
        }
    }
    EC_belong_behind = 1 - EC_belong_front;

    UP = (  E_belong_front              * Kp_rule_list[Pn][Dn]      + 
            E_belong_behind             * Kp_rule_list[Pn + 1][Dn]  + 
            EC_belong_front             * Kp_rule_list[Pn][Dn + 1]  + 
            EC_belong_behind            * Kp_rule_list[Pn + 1][Dn + 1]) / 2;

    return UP;
}

//传入参数误差和误差变化率
float FuzzyPID::_D(float E, float EC) {
    int Pn1 = 0, Dn1 = 0;
    //    //输出量即根据隶属度可能取得的P
    float UD = 0;
    // //    //模糊规则表的制定：
    // float Kd_rule_list[7][7] = {
    //     {PB1, PB1, PB1, PB1, PM1, PS1, ZO1},
    //     {PB1, PB1, PB1, PM1, PM1, ZO1, ZO1},
    //     {PB1, PM1, PM1, PS1, ZO1, NS1, NM1},
    //     {PM1, PM1, PS1, ZO1, NS1, NM1, NM1},
    //     {PS1, PS1, ZO1, NM1, NM1, NM1, NB1},
    //     {ZO1, ZO1, ZO1, NM1, NB1, NB1, NB1},
    //     {ZO1, NS1, NB1, NB1, NB1, NB1, NB1}
    // };


    //    //E的隶属度求法
    //    ///
    //    ///
    //
    float E_belong_front1 = 0; float E_belong_behind1 = 0;
    if (E > error[0] && E < error[6]) {
        if (E < error[1]) {
            Pn1 = 0;
            E_belong_front1 = (error[1] - E) / (error[1] - error[0]);
        }
        else if (E < error[2]) {
            Pn1 = 1;
            E_belong_front1 = (error[2] - E) / (error[2] - error[1]);
        }
        else if (E < error[3]) {
            Pn1 = 2;
            E_belong_front1 = (error[3] - E) / (error[3] - error[2]);
        }
        else if (E < error[4]) {
            Pn1 = 3;
            E_belong_front1 = (error[4] - E) / (error[4] - error[3]);
        }
        else if (E < error[5]) {
            Pn1= 4;
            E_belong_front1 = (error[5] - E) / (error[5] - error[4]);
        }
        else if (E < error[6]) {
            Pn1 = 5;
            E_belong_front1 = (error[6] - E) / (error[6] - error[5]);
        }
        E_belong_behind1 = 1 - E_belong_front1;
    }


    //EC的隶属度求法
    float EC_belong_front1 = 0; float EC_belong_behind1 = 0;

    if (EC > error_c[0] && EC < error_c[6]) {
        if (EC < error_c[1]) {
            Dn1 = 0; EC_belong_front1 = (error_c[1] - EC) / (error_c[1] - error_c[0]);
        }
        else if (EC < error_c[2]) {
            Dn1 = 1; EC_belong_front1 = (error_c[2] - EC) / (error_c[2] - error_c[1]);
        }
        else if (EC < error_c[3]) {
            Dn1 = 2;
            EC_belong_front1 = (error_c[3] - EC) / (error_c[3] - error_c[2]);
        }
        else if (EC < error_c[4]) {
            Dn1 = 3;
            EC_belong_front1 = (error_c[4] - EC) / (error_c[4] - error_c[3]);
        }
        else if (EC < error_c[5]) {
            Dn1= 4;
            EC_belong_front1 = (error_c[5] - EC) / (error_c[5] - error_c[4]);
        }
        else if (EC < error_c[6]) {
            Dn1 = 5;
            EC_belong_front1 = (error_c[6] - EC) / (error_c[6] - error_c[5]);
        }
    }
    EC_belong_behind1 = 1 - EC_belong_front1;
    
    UD = (  E_belong_front1             * Kd_rule_list[Pn1][Dn1]        + 
            E_belong_behind1            * Kd_rule_list[Pn1 + 1][Dn1]    + 
            EC_belong_front1            * Kd_rule_list[Pn1][Dn1 + 1]    + 
            EC_belong_behind1           * Kd_rule_list[Pn1 + 1][Dn1 + 1]) / 2;

    return UD;
}