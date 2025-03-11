#include "../code/camera_param.h"
#include "../src/main.h"
#include <vector>
#include <numeric>
#include <cmath>

#include "../code/common.h"
#include "../code/detection.hpp" // Ai模型预测


using namespace std;
using namespace cv;

bool RACING_ENABLE = false;
bool RACING_FIND = false;
extern float mid_x;
extern float mid_y;
extern int RACING_STOP_COUNT;
/**
 * @brief 追逐区AI识别与路径规划类
 *
 */
class Racing {
public:
    int state = 0;	        // 串口追逐区状态(uart.h 引用 : (extern int ai_state 赋值于main AI_ENABLE) 0 NONE 440 左挡车 -440 右挡车 
    int CAR_Y_TOP = 75;    // 锥桶录入起始行
    // int CAR_Y_DOWN = 220;  // 锥桶录入终止行
    int CAR_SLOW_TOP = 15;  // 车辆急减速判是
    int CAR_SLOW_DOWN = 65; // 车辆急减速判否
    int mid_L_ = 157;       // 车辆靠左，向右侧避让
    int mid_R_ = 176;       // 车辆靠右，向左侧避让
    int danger_L = 150;     // 危险车辆在右侧，右撞车
    int danger_R = 172;     // 危险车辆在左侧，左撞车
    int mid_L = 143;        // 右挡车
    int mid_R = 181;        // 左挡车
    int stop_count = RACING_STOP_COUNT;
    enum RacingType{
        NONE = 0,
        CHECKING,
        SAFE,
        DANGER,
        SUSPICION,
        END,
        NUM
    };
    RacingType racing_type = RacingType::NONE;
    const char* racing_type_name[RacingType::NUM] = {
        "RACING_NONE",
        "RACING_CHECKING"
        "RACING_SAFE",
        "RACING_DANGER",
        "RACING_SUSPICION"
        "RACING_END"
    };
    void _init_(int racing_stop_count,  int avoid_left_car, int avoid_right_car,  int crush_left_car, int crush_right_car,  int stop_left_car, int stop_right_car) {
        this->stop_count = racing_stop_count;
        this->mid_L_ = avoid_left_car;
        this->mid_R_ = avoid_right_car;
        this->danger_L = crush_right_car;
        this->danger_R = crush_left_car;
        this->mid_L = stop_right_car;
        this->mid_R = stop_left_car;

        printf("追逐区初始化...\r\n");
        printf("避让左/右侧车辆(mid_L_, mid_R_) : (%d, %d)\r\n", this->mid_L_, this->mid_R_);
        printf("撞击左/右侧车辆(danger_R, danger_L) : (%d, %d)\r\n", this->danger_R, this->danger_L);
        printf("拦截左/右侧车辆(mid_R, mid_L) : (%d, %d)\r\n", this->mid_R, this->mid_L);
        printf("拦车场次：%d\r\n", this->stop_count);
        printf("追逐区初始化完成！\r\n");

    }
    void reset() {
        mid_x = MIDX_MID;
        enable = false;    // 场景检测使能标志
        none_count = 0;
        racing_count = 0;   // 场次
        safe_count = 0;         // 判安全车辆场次
        danger_count = 0;       // 判危险车辆场次
        suspicion_count = 0;    // 判嫌疑车辆场次
        isMove = false;
        moveleft_count = 0;
        moveright_count = 0;
        moveToLeft = false;
        
        state = 0;
        racing_type = RacingType::NONE;
        car = PredictResult();
        safecar = PredictResult();
        dangercar = PredictResult();
        suspicioncar = PredictResult();
    }
    /**
     * @brief 追逐区AI识别与路径规划处理
     *
     * @param track 赛道识别结果
     * @param predict AI检测结果
     * @return true
     * @return false
     */
    bool process(vector<PredictResult> predict) {
        enable = false; // 场景检测使能标志
        switch(racing_type) {
            case RacingType::NONE:
                for (int i = 0; i < predict.size(); i++){
                    if(predict[i].type == LABEL_CAR) {
                        // racing_count++;
                        // RACING_ENABLE = true;
                        // break;

                        racing_count++;
                        RACING_FIND = true;
                        state = 1;
                        break;
                    }
                }

                if(racing_count >= 2) {
                    // none_count = 0;
                    // racing_count = 0;
                    // RACING_ENABLE = true;
                    // racing_type = RacingType::CHECKING;
                    // state = 1;

                    RACING_ENABLE = true;
                    RACING_FIND = false;
                    none_count = 0;
                    racing_count = 0;
                    state = 1;
                    racing_type = RacingType::CHECKING;
                }

                if(racing_count == 1 && none_count++ >= 10) {
                    COUT1("racing_count == 1 && none_count++ >= 20");
                    none_count = 0;
                    racing_count = 0;
                    // buzzer = 0;
                    // mid_x = MIDX_MID;
                    state = 0;
                    AI_ENABLE = RACING_ENABLE = false;
                    RACING_FIND = false;
                    racing_type = RacingType::NONE;
                }

                break;
            case RacingType::CHECKING: {
                vector<PredictResult> cars;     // 锥桶AI检测数据
                vector<PredictResult> car_safe;     // 锥桶AI检测数据
                vector<PredictResult> car_danger;   // 锥桶AI检测数据
                vector<PredictResult> car_suspicion;// 锥桶AI检测数据
                /*
                LABEL_CAR           // AI标签：车
                LABEL_CARSAFE       // AI标签：安全车辆
                LABEL_CARDANGER     // AI标签：危险车辆
                LABEL_CARSUS        // AI标签：嫌疑车辆
                */
                for(auto i : predict) {
                    if(i.type == LABEL_CAR)         cars.push_back(i);
                    if(i.y + i.height >= CAR_Y_TOP) {
//                        if(i.type == LABEL_CAR)         cars.push_back(i);
                        if(i.type == LABEL_CARSAFE)     car_safe.push_back(i);
                        if(i.type == LABEL_CARDANGER)   car_danger.push_back(i);
                        if(i.type == LABEL_CARSUS)      car_suspicion.push_back(i);
                    }
                }
                if(!cars.empty()) {
                    car = cars[0];
                    // if(car.y + car.height >= CAR_SLOW_TOP && car.y + car.height <= CAR_SLOW_DOWN)  {
                    //     state = 10;
                    // }else{
                    //     // state = -10;
                    // }

                    // 障碍物方向判定（左/右）
                    double ldx = 0, rdx = 0;
                    pair<double, double> p = objDistance(car, 120);
                    ldx = p.first;
                    rdx = p.second;

                    if(!isMove) {
                        // 左侧车辆
                        if(abs(ldx) > 0 && abs(ldx) < abs(rdx)) {
                            moveright_count++;
                        }
                        // 右侧车辆
                        else if(abs(rdx) > 0 && abs(rdx) < abs(ldx)) {
                            moveleft_count++;
                        }
                    }

                    if(moveright_count >= 3 && moveright_count > moveleft_count) {
                        isMove = true;
                        moveleft_count = 0;
                        state = 100;
                        mid_x = mid_L_;
                        COUT2("[L] 车辆检查：靠左",ldx);
                        buzzer = 1;
                        track_type = TRACK_RIGHT;
                        racing_count = 0;
                    }

                    else if(moveleft_count >= 3 && moveleft_count > moveright_count) {
                        isMove = true;
                        moveright_count = 0;
                        state = -100;
                        mid_x = mid_R_;
                        COUT2("[R] 车辆检查：靠右",rdx);
                        buzzer = 1;
                        track_type = TRACK_LEFT;
                        racing_count = 0;
                    }

                    if(car_safe.size() > car_danger.size() && car_safe.size() > car_suspicion.size()) {
                        if(!car_safe.empty())safecar = car_safe[0];
                        safe_count++;
                        danger_count = suspicion_count = 0;
                        car_danger.clear();
                        car_suspicion.clear();
                    }

                    else if(car_danger.size() > car_safe.size() && car_danger.size() > car_suspicion.size()) {
                        if(!car_danger.empty())dangercar = car_danger[0];
                        danger_count++;
                        safe_count = suspicion_count = 0;
                        car_safe.clear();
                        car_suspicion.clear();
                    }

                    else if(car_suspicion.size() > car_danger.size() && car_suspicion.size() > car_safe.size()) {
                        if(!car_suspicion.empty())suspicioncar = car_suspicion[0];
                        suspicion_count++;
                        safe_count = danger_count = 0;
                        car_safe.clear();
                        car_danger.clear();
                    }
                    
                    if(safe_count >= 2) {
                        isMove = false;
                        moveleft_count = moveright_count = 0;
                        safe_count = danger_count = suspicion_count = 0;
                        racing_type = RacingType::SAFE;
                        state = 2;
                        racing_count = 0;
                    }
                    else if(danger_count >= 2) {
                        isMove = false;
                        moveleft_count = moveright_count = 0;
                        safe_count = danger_count = suspicion_count = 0;
                        racing_type = RacingType::DANGER;
                        state = 3;
                        racing_count = 0;
                    }
                    else if(suspicion_count >= 2) {
                        isMove = false;
                        moveleft_count = moveright_count = 0;
                        safe_count = danger_count = suspicion_count = 0;
                        racing_type = RacingType::SUSPICION;
                        state = 4;
                        racing_count = 0;
                    }


                }
                
                else {
                    if(state != 1) {
                        if (racing_count++ >= 10) {
                            racing_count = 0;
                            buzzer = 0;
                            mid_x = MIDX_MID;
                            state = 0;
                            AI_ENABLE = RACING_ENABLE = false;
                            racing_type = RacingType::NONE;
                        }
                    }else {
                        if (racing_count++ >= 20) {
                            racing_count = 0;
                            buzzer = 0;
                            mid_x = MIDX_MID;
                            state = 0;
                            AI_ENABLE = RACING_ENABLE = false;
                            racing_type = RacingType::NONE;
                        }
                    }
                }
            }
                break;
            // 安全车辆，执行绕行
            case RacingType::SAFE: {
                vector<PredictResult> cars;     // 锥桶AI检测数据
                for(auto i : predict) {
                    if(i.type == LABEL_CAR || i.type == LABEL_CARSAFE)         cars.push_back(i);
                }
                if(!cars.empty()) {
                    racing_count = 0;
                    car = cars[0];

                    // 障碍物方向判定（左/右）
                    double ldx = 0, rdx = 0;
                    pair<double, double> p = objDistance(car);
                    ldx = p.first;
                    rdx = p.second;

                    if(!isMove) {
                        // 左侧锥桶
                        if(abs(ldx) > 0 && abs(ldx) < abs(rdx)) {
                            moveright_count++;
                        }
                        // 右侧锥桶
                        else if(abs(rdx) > 0 && abs(rdx) < abs(ldx)) {
                            moveleft_count++;
                        }
                    }

                    if(moveright_count > 2 && moveright_count > moveleft_count) {
                        isMove = true;
                        moveleft_count = 0;
                        state = 20;
                        mid_x = mid_L_;
                        COUT2("[L] 障碍物靠左",ldx);
                        buzzer = 1;
                        track_type = TRACK_RIGHT;
                    }

                    else if(moveleft_count > 2 && moveleft_count > moveright_count) {
                        isMove = true;
                        moveright_count = 0;
                        state = -20;
                        mid_x = mid_R_;
                        COUT2("[R] 障碍物靠右",rdx);
                        buzzer = 1;
                        track_type = TRACK_LEFT;
                    }

                }
                if(state != 1) {
                    if (cars.empty() && racing_count++ >= 3) {
                        racing_count = 0;
                        buzzer = 0;
                        mid_x = MIDX_MID;
                        state = 0;
                        AI_ENABLE = RACING_ENABLE = false;
                        racing_type = RacingType::NONE;
                    }
                }
            }
                break;

            // 危险车辆，执行撞击
            case RacingType::DANGER: {
                vector<PredictResult> cars;     // 锥桶AI检测数据
                for(auto i : predict) {
                    if(i.type == LABEL_CAR || i.type == LABEL_CARDANGER)         cars.push_back(i);
                }
                if(!cars.empty()) {
                    car = cars[0];
                    
                    // 障碍物方向判定（左/右）
                    double ldx = 0, rdx = 0;
                    pair<double, double> p = objDistance(car);
                    ldx = p.first;
                    rdx = p.second;

                    if(!isMove) {
                        // 左侧锥桶
                        if(abs(ldx) > 0 && abs(ldx) < abs(rdx)) {
                            moveleft_count++;
                        }
                        // 右侧锥桶
                        else if(abs(rdx) > 0 && abs(rdx) < abs(ldx)) {
                            moveright_count++;
                        }
                    }

                    // 右侧车辆
                    if(moveright_count >= 2 && moveright_count > moveleft_count) {
                        moveToLeft = false;
                        isMove = true;
                        moveleft_count = 0;
                        track_type = TRACK_RIGHT;
                        racing_count = 0;
                    }
                    // 左侧车辆
                    else if(moveleft_count >= 2 && moveleft_count > moveright_count) {
                        moveToLeft = true;
                        isMove = true;
                        moveright_count = 0;
                        track_type = TRACK_LEFT;
                        racing_count = 0;
                    }

                    // 右侧车辆
                    if(!isMove && moveright_count == 1) {
                        moveToLeft = false;
                        isMove = true;
                    }
                    // 左侧车辆
                    if(!isMove && moveleft_count == 1) {
                        moveToLeft = true;
                        isMove = true;
                    }

                    // 左侧车辆
                    if(moveToLeft) {
                        COUT1("[L] 左撞车");
                        track_type = TRACK_RIGHT;
                        buzzer = 1;
                        state = 330;
                        mid_x = danger_R;
                        isMove = true;
                    }
                    // 右侧车辆
                    else{
                        COUT1("[R] 右撞车");
                        track_type = TRACK_LEFT;
                        buzzer = 1;
                        state = -330;
                        mid_x = danger_L;
                        isMove = true;
                    }
                }else{
                    if(racing_count++ >= 3) {
                        isMove = false;
                        moveToLeft = false;
                        moveright_count = moveleft_count = 0;
                        danger_count = 0;
                        racing_count = 0;
                        buzzer = 0;
                        state = 0;
                        AI_ENABLE = RACING_ENABLE = false;
                        racing_type = RacingType::NONE;
                    }  
                }
                // if(state == 330 || state == -330) {
                //     if(danger_count++ > 4) {
                //         state = 3000;
                //     }
                // }
                        
            }
                break;
            // 嫌疑车辆，执行拦截
            case RacingType::SUSPICION: {
                vector<PredictResult> cars;     // 锥桶AI检测数据
                for(auto i : predict) {
                    if((i.type == LABEL_CAR || i.type == LABEL_CARSUS))         cars.push_back(i);
                }
                if(!cars.empty()) {
                    racing_count = 0;
                    car = cars[0];

                    // 障碍物方向判定（左/右）
                    double ldx = 0, rdx = 0;
                    pair<double, double> p = objDistance(car);
                    ldx = p.first;
                    rdx = p.second;

                    if(!isMove) {
                        // 左侧锥桶
                        if(abs(ldx) > 0 && abs(ldx) < abs(rdx)) {
                            moveright_count++;
                        }
                        // 右侧锥桶
                        else if(abs(rdx) > 0 && abs(rdx) < abs(ldx)) {
                            moveleft_count++;
                        }
                    }

                    // 右侧车辆
                    if(moveright_count >= 2 && moveright_count > moveleft_count) {
                        moveToLeft = true;
                        isMove = true;
                        moveleft_count = 0;
                        state = 40;
                        mid_x = mid_L_;
                        COUT2("[L] 障碍物靠左",ldx);
                        buzzer = 1;
                        track_type = TRACK_RIGHT;
                        racing_count = 0;
                    }
                    // 左侧车辆
                    else if(moveleft_count >= 2 && moveleft_count > moveright_count) {
                        moveToLeft = false;
                        isMove = true;
                        moveright_count = 0;
                        state = -40;
                        mid_x = mid_R_;
                        COUT2("[R] 障碍物靠右",rdx);
                        buzzer = 1;
                        track_type = TRACK_LEFT;
                        racing_count = 0;
                    }
                    none_count = 0;
                }
                // 看不见车辆
                else {
                    // 右侧车辆
                    if(!isMove && moveright_count == 1) {
                        moveToLeft = true;
                        isMove = true;
                    }
                    // 左侧车辆
                    if(!isMove && moveleft_count == 1) {
                        moveToLeft = false;
                        isMove = true;
                    }
                    // 超车
                    if(state == 40 || state == -40) {
                        if(racing_count++ >= 1) {
                            suspicion_count = 0;
                            racing_count = 0;
                            state = 400;
                        }
                    }
                    // 拦截
                    else if(state == 400 || state == 440 || state == -440) {
                        if(isMove) {
                            if(moveToLeft) {
                                COUT1("[L] 左挡车");
                                track_type = TRACK_RIGHT;
                                state = 440;
                                mid_x = mid_R;
                            }
                            else{
                                COUT1("[R] 右挡车");
                                track_type = TRACK_LEFT;
                                state = -440;
                                mid_x = mid_L;
                            }
                        }
                        // 左挡车
                        if(mid_x == mid_R) {
                            if(suspicion_count++ > 10) {
                                if(moveToLeft) {
                                    state = 4000;
                                }
                                else{
                                    state = -4000;
                                }
                                racing_count = suspicion_count = 0;
                            }
                        }
                        // 右挡车
                        else if(mid_x == mid_L) {
                            if(suspicion_count++ > 10) {
                                if(moveToLeft) {
                                    state = 4000;
                                }
                                else{
                                    state = -4000;
                                }
                                racing_count = suspicion_count = 0;
                            }
                        }
                    }
                    // 拦截等待，回归原始寻线
                    else if (state == 4000 || state == -4000) {
                        if(racing_count++ > stop_count) {
                            isMove = false;
                            moveToLeft = false;
                            moveright_count = moveleft_count = 0;
                            suspicion_count = 0;
                            racing_count = 0;
                            buzzer = 0;
                            state = 0;
                            AI_ENABLE = RACING_ENABLE = false;
                            racing_type = RacingType::NONE;
                        }
                    }
                    if(state == 4 && none_count++ >= 40) {
                        COUT1("racing.state == 4 && none_count++ >= 40");
                        none_count = 0;
                        racing_count = 0;
                        buzzer = 0;
                        mid_x = MIDX_MID;
                        state = 0;
                        AI_ENABLE = RACING_ENABLE = false;
                        RACING_FIND = false;
                        racing_type = RacingType::NONE;
                    }
                }            
            }
                break;
        }

        
        return enable;
    }


    /**
     * @brief 图像绘制追逐区识别结果
     *
     * @param img 需要叠加显示的图像
     */
    void drawImage(Mat &img)
    {
        if (enable)
        {
        }
    }

private:
    pair<double, double> objDistance(PredictResult obj, const int num = 80) {
        double obj_poi_x = mapx[(int)(obj.y + obj.height/2)][(int)(obj.x + obj.width/2.0)];
        double obj_poi_y = mapy[(int)(obj.y + obj.height/2)][(int)(obj.x + obj.width/2.0)];
        // COUT2("obj.y + obj.height/2:",obj.y + obj.height/2);

        double ldx = 0;
        for (int i = 0; i < MIN(rpts0s_num, num); i++) {
            if((int)obj_poi_y == (int)rpts0s[i][1] || (int)obj_poi_y - 1 == (int)rpts0s[i][1] || (int)obj_poi_y + 1 == (int)rpts0s[i][1]) {
                ldx = obj_poi_x - rpts0s[i][0];
                break;
            }
        }

        double rdx = 0;
        for (int i = 0; i < MIN(rpts1s_num, num); i++) {
            if((int)obj_poi_y == (int)rpts1s[i][1] || (int)obj_poi_y - 1 == (int)rpts1s[i][1] || (int)obj_poi_y + 1 == (int)rpts1s[i][1]) {
                rdx = rpts1s[i][0] - obj_poi_x;
                break;
            }
        }
        return make_pair(ldx, rdx);
    }
    bool enable = false;    // 场景检测使能标志
    int none_count = 0;
    int racing_count = 0;   // 场次
    int safe_count = 0;         // 判安全车辆场次
    int danger_count = 0;       // 判危险车辆场次
    int suspicion_count = 0;    // 判嫌疑车辆场次
    bool isMove = false;
    int moveleft_count = 0;
    int moveright_count = 0;
    bool moveToLeft = false;
    PredictResult car;
    PredictResult safecar;
    PredictResult dangercar;
    PredictResult suspicioncar;
};
