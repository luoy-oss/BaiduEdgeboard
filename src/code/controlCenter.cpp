#include "../src/main.h"
#include "../recognition/circle_recognition.h"
#include "../recognition/cross_recognition.h"
#include "../recognition/garage_recognition.h"
#include "../recognition/track_recognition.h"
#include "./controlCenter.h"


Speed speed;

double angle;

// 舵机偏差计算变量
double bias_p = 0.0;
double bias_p_last = 0.0;
double bias_d = 0.0;
double ctr = 0.0;
double midAdd = 0.0;
float mid_x = 160.906;
float mid_y = 236.752;
// float mid_x = 161;
// float mid_y = 228;

double P1 = 0;
double P2 = 0;
double D = 0;

SPEED_MODE speed_mode;
int speed_pid_mode;// 下位机清除i积分标志
TRACK_STATE track_state;
/**
 * @brief 智能车方向控制解算
 * 
 * @return PWM 舵机控制PWM
*/
int ControlCenter::smotorSolution(const int danger_state, const int racing_state) {
    speed_pid_mode = 0;
    if(SPEED_DANGER >= 20) {
        if(BLOCK_POI == 'L') {
            track_type = TRACK_RIGHT;
        }else if(BLOCK_POI == 'R') {
            track_type = TRACK_LEFT;
        }
    }
    // 中线跟踪
    if (cross_type != CROSS_HALF_LEFT && cross_type != CROSS_HALF_RIGHT) {
        if (track_type == TRACK_LEFT) {
            rpts = rptsc0;
            rpts_num = rptsc0_num;
        }
        else {
            rpts = rptsc1;
            rpts_num = rptsc1_num;
        }
    }
    else {
        //十字根据远线控制
        if (track_type == TRACK_LEFT) {
            rpts = far_rpts0s;
            track_leftline(far_rpts0s + far_Lpt0_rpts0s_id, far_rpts0s_num - far_Lpt0_rpts0s_id, rpts,
                (int)round(angle_dist / sample_dist), pixel_per_meter * ROAD_WIDTH / 2);
            rpts_num = clip(far_rpts0s_num - far_Lpt0_rpts0s_id, 0, far_rpts0s_num);
        }
        else {
            rpts = far_rpts1s;
            track_rightline(far_rpts1s + far_Lpt1_rpts1s_id, far_rpts1s_num - far_Lpt1_rpts1s_id, rpts,
                (int)round(angle_dist / sample_dist), pixel_per_meter * ROAD_WIDTH / 2);
            rpts_num = clip(far_rpts1s_num - far_Lpt1_rpts1s_id, 0, far_rpts1s_num);
        }
    }

    // 车轮对应点(纯跟踪起始点)
    // for (int r = 0; r < ROWSIMAGE; r++) {
    // 	if ((int)mapx[COLSIMAGE / 2][r] == 161) {
    // 		COUT2("r:" + to_string(r), mapy[COLSIMAGE / 2][r]);
    // 	}
    // }
    // COUT2("mid_x",mid_x);
    if(danger_state == 1 && racing_state == 0) {
        /*
        */
        // 黑块在右， 第一个锥桶位置在右，提前贴左边
        if(BLOCK_POI == 'R') {
            mid_x = MID_R;  // MID_R 锥桶在右时的偏移
        }
        // 黑块在左， 第一个锥桶位置在左，提前贴左边
        else if(BLOCK_POI == 'L'){
            mid_x = MID_L;  // MID_L 锥桶在左时的偏移
        }else{
            COUT2("ERR: 黑块参数错误！ ： ", BLOCK_POI);
        }
    }
    double cx = mid_x;//161;//mapx[COLSIMAGE / 2][164]/*为161.525*/;
    double cy = mid_y;//230;
    // draw_x(&img_line, cx, cy, 2, 255);
    // 找最近点(起始点中线归一化)
    double min_dist = 1e10;
    int begin_id = 0;
    for (int i = 0; i < rpts_num; i++) {
        double dx = rpts[i][0] - cx;
        double dy = rpts[i][1] - cy;
        double dist = sqrt(dx * dx + dy * dy);
        if (dist < min_dist) {
            min_dist = dist;
            begin_id = i;
        }
    }

    // 特殊模式下，不找最近点(由于边线会绕一圈回来，导致最近点为边线最后一个点，从而中线无法正常生成)
    if (garage_type == GARAGE_IN || cross_type != CROSS_NONE /*原为：cross_type == CROSS_IN*/) begin_id = 0;
    
    // 中线有点，同时最近点不是最后几个点
    if (begin_id >= 0 && rpts_num - begin_id >= 3) {
        // 归一化中线
        rpts[begin_id][0] = cx;
        rpts[begin_id][1] = cy;
        rptsn_num = sizeof(rptsn) / sizeof(rptsn[0]);
        resample_points(rpts + begin_id, rpts_num - begin_id, rptsn, &rptsn_num, sample_dist * pixel_per_meter);
        if(is_straight0 && is_straight1 && !AI_ENABLE){
            aim_distance = 1.18;
        }
        if(circle_type == CIRCLE_LEFT_RUNNING || circle_type == CIRCLE_LEFT_OUT ||
        circle_type == CIRCLE_RIGHT_RUNNING || circle_type == CIRCLE_RIGHT_OUT) {
            aim_distance = 0.72;
        }
        if(SPEED_DANGER < 20 && danger_state == 1) {
            aim_distance = 1.18;
        }
        if(BRIDGE_ENABLE) {
            aim_distance = 0.48;
        }
        // 远预锚点位置
        int aim_idx = clip(round(aim_distance / sample_dist), 0, rptsn_num - 1);
#ifdef CAR_DEBUG
        draw_o(&img_line, cx, cy, 3, 255);
        draw_x(&img_line, rptsn[aim_idx][0], rptsn[aim_idx][1], 3, 255);
#endif
        // 计算远锚点偏差值
        double dx = rptsn[aim_idx][0] - cx;
        double dy = cy - rptsn[aim_idx][1] + 0.3 * pixel_per_meter;
        double dn = sqrt(dx * dx + dy * dy);
        double error = -atan2f(dx, dy) * 180 / PI;
        assert(!isnan(error));
        
        double pure_angle = atanl(pixel_per_meter * 2 * 0.3 * dx / dn / dn) / PI * 180 / SMOTOR_RATE;
        
        if(BRIDGE_ENABLE || (Lradius_far > 100 && Rradius_far > 100 && !AI_ENABLE)/*is_straight0 && is_straight1*/) {
            angle = pid_solve(&servo_pid_straight, pure_angle);
            bias_p = servo_pid_straight.out_p;
            bias_d = servo_pid_straight.out_d;
            servo_pid.out_p = bias_p;
            servo_pid.out_d = bias_d;

            angle = MINMAX(angle, -28.5, 28.5);         // 91.5 PWM限幅3600 - 5500
            midAdd = servo_duty(STRAIGHT_SMOTOR_CENTER - angle);

        }else{
            // AdaptivePID pid(servo_pid.kp, , servo_pid.kd, 0.1, 0.01);
            // angle = pid.calculate(pure_angle, 0, 0.01);
            angle = pid_solve(&servo_pid, pure_angle);
            bias_p = servo_pid.out_p;
            bias_d = servo_pid.out_d;
            servo_pid_straight.out_p = bias_p;
            servo_pid_straight.out_i = 0;
            servo_pid_straight.out_d = bias_d;

            // // float k1 = 2.35;
            // // float k2 = 1.3;
            // // float k3 = 3.94;

            // servo_pid.kp = _K1 * log10f(_K2 * abs(pure_angle) + _K3);

            // // double a = 0.54;
            // // double b = 2.5;
            // // double c = 0.24;

            // // servo_pid.kp = a + b * (1 - exp(-c*(abs(pure_angle))));
            // angle = pid_solve(&servo_pid, pure_angle);
            // bias_p = servo_pid.out_p;
            // bias_d = servo_pid.out_d;
            // servo_pid_straight.out_p = bias_p;
            // servo_pid_straight.out_i = 0;
            // servo_pid_straight.out_d = bias_d;

            // angle = MINMAX(angle, -27, 27);         // PWM限幅3600 - 5400
            // angle = MINMAX(angle, -28.5, 28.5);     // PWM限幅3500 - 5400
            // angle = MINMAX(angle, -31.5, 31.5);         // PWM限幅3400 - 5500
            angle = MINMAX(angle, -28.5, 28.5);         // 91.5 PWM限幅3600 - 5500
            // angle = MINMAX(angle, -31, 31);         // PWM限幅3400 - 5500
            midAdd = servo_duty(SMOTOR_CENTER - angle);
        }
        

        extern bool OUT_NONE_LINE;
        if(OUT_NONE_LINE && circle_type == CIRCLE_LEFT_OUT && !is_straight1) {
            COUT1("L _ ERRRRR");
            midAdd = 5500;
        }

        if (OUT_NONE_LINE && circle_type == CIRCLE_RIGHT_OUT && !is_straight0) {
            COUT1("R _ ERRRRR");
            midAdd = 3400;
        }
        // COUT2("danger_state: ",danger_state);
        // COUT2("racing_state: ",racing_state);
        // COUT2("mid_x: ",mid_x);
        
        if(danger_state == 10 || danger_state == 20){
            if(SPEED_DANGER >= 20/*MID_L == MID_R && BLOCK_MID_L == BLOCK_MID_R && MID_L == BLOCK_MID_L*/) {
                // aim_distance = AIM_DISTANCE;//冲刺过危险区参数，此时黑块在左158   黑块在右170，speed32可以
                aim_distance = 1.18;//冲刺过危险区参数，此时黑块在左158   黑块在右170，speed32可以
                double dx = rptsn[aim_idx][0] - cx;
                double dy = cy - rptsn[aim_idx][1] + 0.3 * pixel_per_meter;
                double dn = sqrt(dx * dx + dy * dy);
                double error = -atan2f(dx, dy) * 180 / PI;
                assert(!isnan(error));
                
                double pure_angle = atanl(pixel_per_meter * 2 * 0.3 * dx / dn / dn) / PI * 180 / SMOTOR_RATE;

                angle = pid_solve(&servo_pid_straight, pure_angle);
                bias_p = servo_pid_straight.out_p;
                bias_d = servo_pid_straight.out_d;
                servo_pid.out_p = bias_p;
                servo_pid.out_d = bias_d;
            }else{
                aim_distance = 0.54;
                int aim_idx = clip(round(aim_distance / sample_dist), 0, rptsn_num - 1);
                double dx = rptsn[aim_idx][0] - cx;
                double dy = cy - rptsn[aim_idx][1] + 0.3 * pixel_per_meter;
                double dn = sqrt(dx * dx + dy * dy);
                double error = -atan2f(dx, dy) * 180 / PI;
                assert(!isnan(error));

                pure_angle = atanl(pixel_per_meter * 2 * 0.3 * dx / dn / dn) / PI * 180 / SMOTOR_RATE;
                angle = pid_2_solve(&servo_pid_danger, pure_angle);
                bias_p = servo_pid_danger.out_p;
                bias_d = servo_pid_danger.out_d;
                servo_pid.out_p = 0;
                servo_pid.out_d = 0;
                servo_pid_straight.out_p = 0;
                servo_pid_straight.out_i = 0;
                servo_pid_straight.out_d = 0;
            }

            // angle = MINMAX(angle, -27, 27);     // PWM限幅3600 - 5400
            angle = MINMAX(angle, -28.5, 28.5); // PWM限幅3500 - 5400
            // angle = MINMAX(angle, -31.5, 31.5);         // PWM限幅3400 - 5500
            midAdd = servo_duty(SMOTOR_CENTER - angle);
        }
        else if(racing_state != 0) {
            aim_distance = 1.18;
            int aim_idx = clip(round(aim_distance / sample_dist), 0, rptsn_num - 1);
            double dx = rptsn[aim_idx][0] - cx;
            double dy = cy - rptsn[aim_idx][1] + 0.3 * pixel_per_meter;
            double dn = sqrt(dx * dx + dy * dy);
            double error = -atan2f(dx, dy) * 180 / PI;
            assert(!isnan(error));

            pure_angle = atanl(pixel_per_meter * 2 * 0.3 * dx / dn / dn) / PI * 180 / SMOTOR_RATE;
            angle = pid_solve(&servo_pid_racing, pure_angle);
            bias_p = servo_pid_racing.out_p;
            bias_d = servo_pid_racing.out_d;
            servo_pid.out_p = 0;
            servo_pid.out_d = 0;
            servo_pid_straight.out_p = 0;
            servo_pid_straight.out_i = 0;
            servo_pid_straight.out_d = 0;

            // angle = MINMAX(angle, -27, 27);     // PWM限幅3600 - 5400
            angle = MINMAX(angle, -28.5, 28.5); // PWM限幅3500 - 5400
            // angle = MINMAX(angle, -31.5, 31.5);         // PWM限幅3400 - 5500
            midAdd = servo_duty(SMOTOR_CENTER - angle);
        }
    }
    else {
        // 中线点过少(出现问题)，则不控制舵机
        extern bool OUT_NONE_LINE;
        if(OUT_NONE_LINE && circle_type == CIRCLE_LEFT_OUT && !is_straight1) {
            COUT1("中线点过少L _ ERRRRR");
            midAdd = 5500;
        }

        if (OUT_NONE_LINE && circle_type == CIRCLE_RIGHT_OUT && !is_straight0) {
            COUT1("中线点过少R _ ERRRRR");
            midAdd = 3400;
        }

        rptsn_num = 0;
    }
    extern int slow_count;
    extern int fast_count;
    
    if(circle_type == CIRCLE_NONE) {
        // 加速状态
        if(track_state == TRACK_STATE::TRACK_FAST) {
            if (cross_type != CROSS_NONE) {
                fast_count = 0;
                slow_count = 0;
                track_state = TRACK_STATE::TRACK_FAST;
            }
            if (fast_count++ > FAST_COUNT) {
                if (Lradius < 100 && Rradius < 100) {
                    fast_count = 0;
                    track_state = TRACK_STATE::TRACK_SLOW;
                }
            }
        }
        // 减速状态
        else {
            // 维持减速状态场次
            if (slow_count++ > SLOW_COUNT) {
                // 退出减速状态
                slow_count = 0;
                track_state = TRACK_STATE::TRACK_FAST;
            }
        }
    }
    
#ifdef CAR_DEBUG
    draw_circle();
    draw_cross();
    // 绘制道路线
    for (int i = 0; i < rpts0s_num; i++) {
        AT_IMAGE(&img_line, clip(rpts0s[i][0], 0, img_line.width - 1),
            clip(rpts0s[i][1], 0, img_line.height - 1)) = 255;
    }
    for (int i = 0; i < rpts1s_num; i++) {
        AT_IMAGE(&img_line, clip(rpts1s[i][0], 0, img_line.width - 1),
            clip(rpts1s[i][1], 0, img_line.height - 1)) = 255;
    }
    for (int i = 0; i < rptsn_num; i++) {
        AT_IMAGE(&img_line, clip(rptsn[i][0], 0, img_line.width - 1),
            clip(rptsn[i][1], 0, img_line.height - 1)) = 255;
    }
    for (int i = 0; i < circle_rptss_num; i++) {
        AT_IMAGE(&img_line, clip(circle_rptss[i][0], 0, img_line.width - 1),
            clip(circle_rptss[i][1], 0, img_line.height - 1)) = 255;
    }
    

    // 绘制锚点
    int aim_idx = clip(round(aim_distance / sample_dist), 0, rptsn_num - 1);
    draw_x(&img_line, rptsn[aim_idx][0], rptsn[aim_idx][1], 3, 255);
    // 绘制角点
    if (Lpt0_found) {
        draw_x(&img_line, rpts0s[Lpt0_rpts0s_id][0], rpts0s[Lpt0_rpts0s_id][1], 7, 255);
    }
    if (Lpt1_found) {
        draw_x(&img_line, rpts1s[Lpt1_rpts1s_id][0], rpts1s[Lpt1_rpts1s_id][1], 7, 255);
    }
    
    if (circle_Lpt_found) {
        draw_x(&img_line, circle_rptss[circle_Lpt_rptss_id][0], circle_rptss[circle_Lpt_rptss_id][1], 5, 255);
    }	
    
    // extern cv::Mat nitoushi;
    // for (int c = 0; c < COLSIMAGE; c++) {
    //     for (int r = 0; r < ROWSIMAGE; r++) {
    //         if (AT_IMAGE(&img_line, c, r) < 140) {
    //             cv::circle(nitoushi, cv::Point(c, r), 1, cv::Scalar(0, 0, 0));
    //         }
    //         else {
    //             cv::circle(nitoushi, cv::Point(c, r), 1, cv::Scalar(255, 255, 255));
    //         }
    //     }
    // }
    // static int i = 1;
    // std::string filename = "../frame/nitoushi" + std::to_string(i++) + ".jpg";
    // imwrite(filename, nitoushi);

#endif

    return midAdd;
}



// 丢线场计数
int OUT_COUNT = 10;

// 短直道场计数
int SHORT_COUNT = 3;

// 急减速场计数
int SLOW_COUNT = 30;

// 加速场计数
int FAST_COUNT = 10;

// 速度
int SPEED_BRIDGE = 20;
int SPEED_DANGER = 10;
int SPEED_RACING = 20;
int SPEED_RESCUE = 10;

int SPEED_NORMAL = 38;
int SPEED_FAST = 38;
int SPEED_SLOW = 30;
int SPEED_SLOWEST = SPEED_SLOW - 15;
int SPEED_ADD = 40;

// 圆环速度
int SPEED_CIRCLE_BEGIN = 26;
int SPEED_CIRCLE_MIN = 25;
int SPEED_CIRCLE_RUNNING = SPEED_CIRCLE_MIN;

// 偏差速度控制值abs(Servo_Center_Mid - midAdd) > BIAS_SPEED_LIMIT后进入慢速控制
int BIAS_SPEED_LIMIT = 150;

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

double AIM_DISTANCE = 0.62;

bool ai_debug = false;      // AI图像显示使能
bool bi_debug = false;      // 二值化图像显示使能
bool track_debug = false;   // 赛道线图像显示使能
bool nts_debug = false;     // 逆透视图像显示使能
// bool danger_debug = false;  // 危险区图像显示使能
bool saveImage = false;     // 存图使能

char BLOCK_POI = 'L';
float MID_L = 153;
float MID_R = 167;
float BLOCK_MID_L = 148;
float BLOCK_MID_R = 172;
int BOMB_Y_TOP = 15;    // 爆炸物录入起始行
int CONE_Y_TOP = 40;    // 锥桶录入起始行
int CONE_Y_DOWN = 165;  // 锥桶录入终止行

float _K1 = 2.35;
float _K2 = 1.3;
float _K3 = 3.94;

int RACING_STOP_COUNT = 130;
int AVOID_LEFT_CAR = 157;
int AVOID_RIGHT_CAR = 176;
int CRUSH_LEFT_CAR = 150;
int CRUSH_RIGHT_CAR = 172; 
int STOP_LEFT_CAR = 143;
int STOP_RIGHT_CAR = 181;


bool it_AI;
bool it_RESCUE;
bool it_RACING;
bool it_DANGER;
bool it_BRIDGE;


void ControlCenter::valueInit(const ValueConfig& valueConfig) {
    P1 = valueConfig.params.servo_pid_P1;
    P2 = valueConfig.params.servo_pid_P2;
    D = valueConfig.params.servo_pid_D;
    //舵机控制PID
    //                   (kp, kp2, kd)
    // servo_pid = pid_param_t(valueConfig.params.servo_pid_P1, valueConfig.params.servo_pid_P2, valueConfig.params.servo_pid_D);
    servo_pid = pid_param_t(valueConfig.params.servo_pid_P1,0,valueConfig.params.servo_pid_D,1,100,100,100);
    servo_pid_straight = pid_param_t(valueConfig.params.servo_pid_P2, 0.003725, valueConfig.params.servo_pid_D/2,1, 100, 4, 100);


    ff_servo_pid = pid_param_t(valueConfig.params.servo_pid_P1,0,valueConfig.params.servo_pid_D, 
                                valueConfig.params.servo_pid_f, 1,100,100,100);

    ff_servo_pid_straight = pid_param_t(valueConfig.params.servo_pid_P2, 0.003725, valueConfig.params.servo_pid_D/2, 
                                valueConfig.params.servo_pid_f/2 ,1, 100, 4, 100);

    _K1 = valueConfig.params.servo_pid_k1;
    _K2 = valueConfig.params.servo_pid_k2;
    _K3 = valueConfig.params.servo_pid_k3;

    servo_pid_danger = pid_param_t(2.78075, 0, 16.4, 1, 100, 100, 100);
    servo_pid_racing = pid_param_t(2.78075, 0, 16.4, 1, 100, 7, 100);

    AIM_DISTANCE = valueConfig.params.aim_distance;// 默认0.62

    // 丢线场计数
    OUT_COUNT = valueConfig.params.out_count;

    // 短直道场计数
    SHORT_COUNT = valueConfig.params.short_count;

    // 急减速场计数
    SLOW_COUNT = valueConfig.params.slow_count;

    // 加速场计数
    FAST_COUNT = valueConfig.params.fast_count;

    if(!valueConfig.params.speed_debug) {
        // 速度
        SPEED_BRIDGE = valueConfig.params.speed_bridge;
        SPEED_DANGER = valueConfig.params.speed_danger;
        SPEED_RACING = valueConfig.params.speed_racing;
        SPEED_RESCUE = valueConfig.params.speed_rescue;

        SPEED_NORMAL = valueConfig.params.speed_normal;
        SPEED_FAST = valueConfig.params.speed_fast;
        SPEED_SLOW = valueConfig.params.speed_slow;
        SPEED_SLOWEST = valueConfig.params.speed_slowest;
        SPEED_ADD = valueConfig.params.speed_add;

        // 圆环速度
        SPEED_CIRCLE_BEGIN = valueConfig.params.speed_circle_begin;
        SPEED_CIRCLE_MIN = valueConfig.params.speed_circle_min;

        SPEED_CIRCLE_RUNNING = valueConfig.params.speed_circle_running;
    }else{
        SPEED_BRIDGE = SPEED_DANGER = SPEED_RACING = SPEED_RESCUE = valueConfig.params.speed_normal;
        SPEED_NORMAL = SPEED_FAST = SPEED_SLOW = SPEED_SLOWEST = SPEED_ADD = valueConfig.params.speed_normal;
        SPEED_CIRCLE_RUNNING = SPEED_CIRCLE_BEGIN = SPEED_CIRCLE_MIN = valueConfig.params.speed_normal;
    }
    // 偏差速度控制值abs(Servo_Center_Mid - midAdd) > BIAS_SPEED_LIMIT后进入慢速控制
    BIAS_SPEED_LIMIT = valueConfig.params.bias_speed_limit;

    LR_num1 = valueConfig.params.LR_num1;
    LR_num2 = valueConfig.params.LR_num2;
    LR_num3 = valueConfig.params.LR_num3;

    RR_num1 = valueConfig.params.RR_num1;
    RR_num2 = valueConfig.params.RR_num2;
    RR_num3 = valueConfig.params.RR_num3;

    LR_F_num1 = valueConfig.params.LR_F_num1;
    LR_F_num2 = valueConfig.params.LR_F_num2;
    LR_F_num3 = valueConfig.params.LR_F_num3;

    RR_F_num1 = valueConfig.params.RR_F_num1;
    RR_F_num2 = valueConfig.params.RR_F_num2;
    RR_F_num3 = valueConfig.params.RR_F_num3;


    ai_debug = valueConfig.params.ai_debug;
    bi_debug = valueConfig.params.bi_debug;
    track_debug = valueConfig.params.track_debug;
    nts_debug = valueConfig.params.nts_debug;
    // danger_debug = valueConfig.params.danger_debug;
    saveImage = valueConfig.params.saveImage;

    BLOCK_POI = std::toupper(valueConfig.params.BLOCK_POI);
    if(BLOCK_POI == 'L') {
        COUT1("黑块位置 >>> 左");
        COUT1("黑块位置 >>> 左");
        COUT1("黑块位置 >>> 左");
        MID_L = valueConfig.params.L_MID_L;
        MID_R = valueConfig.params.L_MID_R;
        BLOCK_MID_L = valueConfig.params.L_BLOCK_MID_L;
        BLOCK_MID_R = valueConfig.params.L_BLOCK_MID_R;

        BOMB_Y_TOP = valueConfig.params.L_BOMB_Y_TOP;    // 爆炸物录入起始行
        CONE_Y_TOP = valueConfig.params.L_CONE_Y_TOP;    // 锥桶录入起始行
        CONE_Y_DOWN = valueConfig.params.L_CONE_Y_DOWN;  // 锥桶录入终止行
    }else if(BLOCK_POI == 'R') {
        COUT1("黑块位置 >>> 右");
        COUT1("黑块位置 >>> 右");
        COUT1("黑块位置 >>> 右");
        MID_L = valueConfig.params.R_MID_L;
        MID_R = valueConfig.params.R_MID_R;
        BLOCK_MID_L = valueConfig.params.R_BLOCK_MID_L;
        BLOCK_MID_R = valueConfig.params.R_BLOCK_MID_R;

        BOMB_Y_TOP = valueConfig.params.R_BOMB_Y_TOP;    // 爆炸物录入起始行
        CONE_Y_TOP = valueConfig.params.R_CONE_Y_TOP;    // 锥桶录入起始行
        CONE_Y_DOWN = valueConfig.params.R_CONE_Y_DOWN;  // 锥桶录入终止行
    }
    
    RACING_STOP_COUNT = valueConfig.params.RACING_STOP_COUNT;
	AVOID_LEFT_CAR = valueConfig.params.AVOID_LEFT_CAR;
	AVOID_RIGHT_CAR = valueConfig.params.AVOID_RIGHT_CAR;
	CRUSH_LEFT_CAR = valueConfig.params.CRUSH_LEFT_CAR;
	CRUSH_RIGHT_CAR = valueConfig.params.CRUSH_RIGHT_CAR; 
	STOP_LEFT_CAR = valueConfig.params.STOP_LEFT_CAR;
	STOP_RIGHT_CAR = valueConfig.params.STOP_RIGHT_CAR;


    it_AI = valueConfig.params.it_AI;
    it_RESCUE = valueConfig.params.it_RESCUE;
    it_RACING = valueConfig.params.it_RACING;
    it_DANGER = valueConfig.params.it_DANGER;
    it_BRIDGE = valueConfig.params.it_BRIDGE;
}
