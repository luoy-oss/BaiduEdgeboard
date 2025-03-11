#ifndef __CONTROL_CENTER_H__
#define __CONTROL_CENTER_H__
#include "../src/main.h"
#include "./pid.h"
#include "./smotor.h"
#include "./value_config.cpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

// 速度控制变量
struct Speed{
    int normal = 27;
    int fast = 30;
    int slow = 24;
    int add = 50;
    Speed(){
        this->normal = 27;
        this->fast = 30;
        this->slow = 24;
        this->add = 50;
    }
};

extern Speed speed;
extern double bias_p;
extern double bias_p_last;
extern double bias_d;
extern double ctr;
extern double midAdd;
extern float mid_x;
extern float mid_y;

extern char BLOCK_POI;
extern float MID_L;
extern float MID_R;
extern float BLOCK_MID_L;
extern float BLOCK_MID_R;
extern int BOMB_Y_TOP;    // 爆炸物录入起始行
extern int CONE_Y_TOP;    // 锥桶录入起始行
extern int CONE_Y_DOWN;  // 锥桶录入终止行

extern float _K1;
extern float _K2;
extern float _K3;

extern int RACING_STOP_COUNT;
extern int AVOID_LEFT_CAR;
extern int AVOID_RIGHT_CAR;
extern int CRUSH_LEFT_CAR;
extern int CRUSH_RIGHT_CAR; 
extern int STOP_LEFT_CAR;
extern int STOP_RIGHT_CAR;

extern bool it_AI;
extern bool it_RESCUE;
extern bool it_RACING;
extern bool it_DANGER;
extern bool it_BRIDGE;

extern int speed_pid_mode;

enum SPEED_MODE{
    SPEED_NONE = 0,
    NORMAL,
    BANGBANG,       //BANGBANG
    SOFT,           //SOFT
};


extern SPEED_MODE speed_mode;


enum TRACK_STATE{
    TRACK_FAST = 1,
    TRACK_SLOW = 2,
};
extern TRACK_STATE track_state;

/**
 * @brief 智能车速度与方向控制类
 * 
 */
class ControlCenter{
public:

    /**
     * @brief 各赛道元素速度参数初始化 
    */
    void valueInit(const ValueConfig&);

    /**
     * @brief 舵机控制解算
    */
    int smotorSolution(const int, const int);

    /**
     * @brief 电机控制解算
    */
    int motorSolution();
private:
    pid_param_t servo_pid_danger;
    pid_param_t servo_pid_racing;
    pid_param_t servo_pid_rescue;
    pid_param_t servo_pid;
    pid_param_t servo_pid_straight;

    pid_param_t ff_servo_pid;   //   前馈pid
    pid_param_t ff_servo_pid_straight;
};
#endif