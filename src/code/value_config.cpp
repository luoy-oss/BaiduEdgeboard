#pragma once
#include "../src/main.h"
#include "json.hpp"
#include <cmath>
#include <fstream>
#include <iostream>

using namespace std;

class ValueConfig {
private:

public:
    struct Params {
        // 丢线场计数
        int out_count = 10;

        // 短直道场计数
        int short_count = 3;

        // 急减速场计数
        int slow_count = 30;

        // 加速场计数
        int fast_count = 30;

        // speed_debug = true时，所有速度均为speed_normal设置的速度
        bool speed_debug = false;
        // 速度
        int speed_bridge = 20;
        int speed_danger = 10;
        int speed_racing = 20;
        int speed_rescue = 10;


        int speed_normal = 38;
        int speed_fast = 38;
        int speed_slow = 30;
        int speed_slowest = speed_slow - 15;
        int speed_add = 40;

        // 圆环速度
        int speed_circle_begin = 26;
        int speed_circle_min = 25;
        int speed_circle_running = speed_circle_min;

        // 偏差速度控制值abs(4500 - midAdd) > BIAS_SPEED_LIMIT后进入慢速控制
        int bias_speed_limit = 150;

        // 逆透视半径计算瞄点
        int LR_num1 = 8;
        int LR_num2 = 16;
        int LR_num3 = 24;

        int RR_num1 = 8;
        int RR_num2 = 16;
        int RR_num3 = 24;

        int LR_F_num1 = 24;
        int LR_F_num2 = 34;
        int LR_F_num3 = 44;
        
        int RR_F_num1 = 24;
        int RR_F_num2 = 34;
        int RR_F_num3 = 44;

        double aim_distance = 0.62;
        double servo_pid_P1 = 1.7;          // 一阶比例系数：直线控制量
        double servo_pid_P2 = 0.1;          // 二阶比例系数：弯道控制量
        double servo_pid_f = 0.1;          // 前馈系数
        double servo_pid_D = 0;             // 一阶比例系数：转弯控制量
        double servo_pid_k1 = 2.25;
        double servo_pid_k2 = 1.34;
        double servo_pid_k3 = 3;

        bool ai_debug = false;      // AI图像显示使能
        bool bi_debug = false;      // 二值化图像显示使能
        bool track_debug = false;   // 赛道线图像显示使能
        bool nts_debug = false;     // 逆透视图像显示使能
        bool saveImage = false;     // 存图使能

        float cone_score = 0.6;
        float bomb_score = 0.6;
        float block_score = 0.6;
        float bridge_score = 0.6;
        float crosswalk_score = 0.6;
        float people_score = 0.6;
        float car_score = 0.75;
        float car_label_score = 0.9;

        char BLOCK_POI = 'L';   // 黑块位置（L / l 左 ，  R / r 右）

        float L_MID_L = 154;
        float L_MID_R = 179;
        float L_BLOCK_MID_L = 158;
        float L_BLOCK_MID_R = 166;
        int L_BOMB_Y_TOP = 15;    // 爆炸物录入起始行
        int L_CONE_Y_TOP = 67;    // 锥桶录入起始行
        int L_CONE_Y_DOWN = 207;  // 锥桶录入终止行

        float R_MID_L = 150;
        float R_MID_R = 170.1;
        float R_BLOCK_MID_L = 158;
        float R_BLOCK_MID_R = 166;
        int R_BOMB_Y_TOP = 15;    // 爆炸物录入起始行
        int R_CONE_Y_TOP = 67;    // 锥桶录入起始行
        int R_CONE_Y_DOWN = 207;  // 锥桶录入终止行

        int RACING_STOP_COUNT = 130;
        int AVOID_LEFT_CAR = 157;
        int AVOID_RIGHT_CAR = 176;
        int CRUSH_LEFT_CAR = 150;
        int CRUSH_RIGHT_CAR = 172; 
        int STOP_LEFT_CAR = 143;
        int STOP_RIGHT_CAR = 181;

        bool it_AI = true;             // AI区域使能
        bool it_RESCUE = true;         // 救援区使能
        bool it_RACING = true;         // 追逐区使能
        bool it_DANGER = true;         // 危险区使能
        bool it_BRIDGE = true;         // 桥使能

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(
            Params, 
            out_count, short_count, slow_count,fast_count,
            speed_debug,
            speed_bridge, speed_danger, speed_racing, speed_rescue,
            speed_normal, speed_fast, speed_slow, speed_slowest, speed_add,
            speed_circle_begin, speed_circle_min, speed_circle_running,
            bias_speed_limit,
            aim_distance,
            servo_pid_P1, servo_pid_P2, servo_pid_f, servo_pid_D, //servo_pid_k1, servo_pid_k2, servo_pid_k3, 
            ai_debug, bi_debug, track_debug, nts_debug, saveImage,
            cone_score,bomb_score,block_score,bridge_score,crosswalk_score,people_score, car_score, car_label_score,
            BLOCK_POI,
            L_MID_L, L_MID_R, L_BLOCK_MID_L, L_BLOCK_MID_R, L_BOMB_Y_TOP, L_CONE_Y_TOP, L_CONE_Y_DOWN,
            R_MID_L, R_MID_R, R_BLOCK_MID_L, R_BLOCK_MID_R, R_BOMB_Y_TOP, R_CONE_Y_TOP, R_CONE_Y_DOWN,
            RACING_STOP_COUNT, AVOID_LEFT_CAR, AVOID_RIGHT_CAR, CRUSH_LEFT_CAR, CRUSH_RIGHT_CAR, STOP_LEFT_CAR, STOP_RIGHT_CAR,
            it_AI, it_RESCUE, it_RACING, it_DANGER, it_BRIDGE); // 添加构造函数
    };

    Params params;                   // 读取控制参数
    /**
     * @brief 加载配置参数Json
     */
    ValueConfig() {
        string jsonPath = "../src/config/value_config.json";
        std::ifstream config_is(jsonPath);
        if (!config_is.good()) {
            std::cout << "Error: Params file path:[" << jsonPath << "] not find .\n";
            exit(-1);
        }

        nlohmann::json js_value;
        config_is >> js_value;

        try {
            params = js_value.get<Params>();
        } catch (const nlohmann::detail::exception &e) {
            std::cerr << "Json Params Parse failed :" << e.what() << '\n';
            exit(-1);
        }
        cout <<"--- out_count: " << params.out_count << endl
             << " | short_count: " << params.short_count << endl
             << " | slow_count: " << params.slow_count << endl;
        cout <<"--- speed_debug: " << (params.speed_debug ? "true" : "false") << endl;
        cout <<"--- speed_normal: " << params.speed_normal << endl;
        // cout <<"--- speed_normal: " << params.speed_normal << endl
        //      << " | speed_fast: " << params.speed_fast << endl
        //      << " | speed_slow: " << params.speed_slow << endl
        //      << " | speed_slowest: " << params.speed_slowest << endl
        //      << " | speed_add: " << params.speed_add << endl;

        // cout <<"--- speed_circle_begin: " << params.speed_circle_begin << endl
        //      << " | speed_circle_min: " << params.speed_circle_min << endl;

        // cout <<"--- bias_speed_limit: " << params.bias_speed_limit << endl;

        cout <<"--- aim_distance: " << params.aim_distance << " >> equal point : " << params.aim_distance / sample_dist << endl;
        // cout << "--- servo_pid_P1:" << params.servo_pid_P1 << " | servo_pid_P2:" << params.servo_pid_P2
        //         << "--- servo_pid_D:" << params.servo_pid_D << endl;
        printf("cone_score : %.2f\t", params.cone_score);
        printf("bomb_score : %.2f\t", params.bomb_score);
        printf("block_score : %.2f\t", params.block_score);
        printf("bridge_score : %.2f\t", params.bridge_score);
        printf("crosswalk_score : %.2f\n", params.crosswalk_score);
        printf("rescue_score : %.2f\n", params.people_score);
        printf("car_score : %.2f\n", params.car_score);
    }
};
