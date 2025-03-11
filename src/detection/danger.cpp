#include "../code/camera_param.h"
#include "../src/main.h"
#include <vector>
#include <numeric>
#include <cmath>

#include "../code/common.h"
#include "../code/detection.hpp" // Ai模型预测


using namespace std;
using namespace cv;

bool DANGER_ENABLE = false;
extern float mid_x;
extern float mid_y;

extern float MID_L;
extern float MID_R;
extern float BLOCK_MID_L;
extern float BLOCK_MID_R;
extern int BOMB_Y_TOP;    // 爆炸物录入起始行
extern int CONE_Y_TOP;    // 锥桶录入起始行
extern int CONE_Y_DOWN;  // 锥桶录入终止行

// #define CAR_DEBUG
// #define CAR_SHOW
/**
 * @brief 危险区AI识别与路径规划类
 *
 */
class Danger
{

public:
    int state = 0;
    // int cone_y_top = 90;    // 锥桶录入起始行
    int bomb_y_top = BOMB_Y_TOP;    // 爆炸物录入起始行
    int cone_y_top = CONE_Y_TOP;    // 锥桶录入起始行
    int cone_y_down = CONE_Y_DOWN;  // 锥桶录入终止行

    char block_poi = 'L';
    float mid_L = MID_L;
    float mid_R = MID_R;
    float block_mid_L = BLOCK_MID_L;
    float block_mid_R = BLOCK_MID_R;

    enum DangerType{
        NONE = 0,
        RUN,
        END,
        NUM
    };
    DangerType danger_type = DangerType::NONE;
    const char* danger_type_name[DangerType::NUM] = {
        "DANGER_NONE",
        "DANGER_RUN",
        "DANGER_END",
    };
    void _init_(char block_poi, int bomb_y_top, int cone_y_top, int cone_y_down, float mid_L,float mid_R, float block_mid_L,float block_mid_R) {
        block_poi = std::toupper(block_poi);
        this->block_poi = block_poi;
        this->bomb_y_top = bomb_y_top;
        this->cone_y_top = cone_y_top;
        this->cone_y_down = cone_y_down;
        if(SPEED_DANGER >= 20) {
            COUT1("SPEED_DANGER >= 20");
            if(this->block_poi == 'L') {
                this->mid_L = mid_L;
                this->mid_R = mid_L;
                this->block_mid_L = mid_L;
                this->block_mid_R = mid_L;
                COUT1(">>>沿右边冲刺");
                COUT1(">>>沿右边冲刺");
                COUT1(">>>沿右边冲刺");
                COUT1(">>>沿右边冲刺");
                COUT1(">>>沿右边冲刺");
            }else if(this->block_poi == 'R') {
                this->mid_L = mid_R;
                this->mid_R = mid_R;
                this->block_mid_L = mid_R;
                this->block_mid_R = mid_R;
                COUT1(">>>沿左边冲刺");
                COUT1(">>>沿左边冲刺");
                COUT1(">>>沿左边冲刺");
                COUT1(">>>沿左边冲刺");
                COUT1(">>>沿左边冲刺");
            }
        }else{
            this->mid_L = mid_L;
            this->mid_R = mid_R;
            this->block_mid_L = block_mid_L;
            this->block_mid_R = block_mid_R;
        }

        COUT2(mid_L, mid_R);
        COUT2(block_mid_L, block_mid_R);

        printf("危险区初始化...\r\n");
        printf("黑块位置：%c\r\n", this->block_poi);
        printf("爆炸物原图判入y坐标 : %d\r\n", this->bomb_y_top);
        printf("锥桶原图判入判出y坐标 : %d， %d\r\n", this->cone_y_top, this->cone_y_down);
        printf("(mid_L, mid_R) : (%.2f, %.2f)\r\n", this->mid_L, this->mid_R);
        printf("(block_mid_L, block_mid_R) : (%.2f, %.2f)\r\n", this->block_mid_L, this->block_mid_R);
        printf("参数检查...\r\n");
        assert(block_poi == 'L' || block_poi == 'R');
        printf("危险区初始化完成！\r\n");
    }
    void reset() {
        danger_type = DangerType::NONE;
    }
    /**
     * @brief 危险区AI识别与路径规划处理
     *
     * @param track 赛道识别结果
     * @param predict AI检测结果
     * @return true
     * @return false
     */
    bool process(vector<PredictResult> predict) {
#ifdef CAR_DEBUG 
        Mat ff = Mat::zeros(cv::Size(320, 240), CV_8UC3);
        Mat danger_frame = Mat::zeros(cv::Size(320, 240), CV_8UC3);
#endif
        enable = false; // 场景检测使能标志
        switch(danger_type) {
            case DangerType::NONE: {
                bool have_cone = false;
                for (int i = 0; i < predict.size(); i++) {
                    if(predict[i].type == LABEL_CONE) {
                        have_cone = true;
                    }
                }
                for (int i = 0; i < predict.size(); i++){
                    if(/*have_cone && */predict[i].type == LABEL_BOMB && predict[i].y + predict[i].height/2 > bomb_y_top) {
                        COUT2("LABEL_BOMB",predict[i].y);
                        danger_type = DangerType::RUN;
                        DANGER_ENABLE = true;
                        state = 1;
                    }
                }
                break;
            }
            case DangerType::RUN: {
                DANGER_ENABLE = true;
                vector<PredictResult> resultsObs; // 锥桶AI检测数据
                vector<PredictResult> cones; // 锥桶AI检测数据
                vector<PredictResult> blocks; // 锥桶AI检测数据
                bool have_bomb = false;
                for (int i = 0; i < predict.size(); i++) {
                    // AI标志距离计算
                    if (predict[i].type == LABEL_CONE /*&& (predict[i].y + predict[i].height) > ROWSIMAGE * 0.4*/)  {
                        resultsObs.push_back(predict[i]);
                    }

                    if(predict[i].type == LABEL_BLOCK && predict[i].y + predict[i].height >= cone_y_top) {
                        blocks.push_back(predict[i]);
                    }
                    if(predict[i].type == LABEL_BOMB) {
                        have_bomb = true;
                    }
                }

                for(const auto& cone : resultsObs) {
                    if(cone.y + cone.height >= cone_y_top && cone.y + cone.height <= cone_y_down){
                        cones.push_back(cone);
#ifdef CAR_DEBUG                     
                        // 锥桶识别中心坐标(逆透视后)
                        double cone_poi_x = mapx[(int)(cone.y + cone.height/2)][(int)(cone.x + cone.width/2.0)];
                        double cone_poi_y = mapy[(int)(cone.y + cone.height/2)][(int)(cone.x + cone.width/2.0)];
                        cv::circle(ff, cv::Point(cone_poi_x, cone_poi_y), 2, Scalar(255,245,0));
#endif
                    }
                }
                static bool last_is_cone = false;
                static bool last_turn_left = true;
                if(!cones.empty()) {
                    // 选取距离最近的锥桶
                    int areaMax = 0; // 框面积
                    int index = 0;   // 目标序号
                    for (int i = 0; i < cones.size(); i++) {
                        int area = cones[i].width * cones[i].height;
                        if (area >= areaMax) {
                            index = i;
                            areaMax = area;
                        }
                    }
                    
                    cone = cones[index];
                    enable = true; // 场景检测使能标志

                    // 障碍物方向判定（左/右）
                    double cone_poi_x = mapx[(int)(cone.y + cone.height / 2.0)][(int)(cone.x + cone.width / 2.0)];
                    double cone_poi_y = mapy[(int)(cone.y + cone.height / 2.0)][(int)(cone.x + cone.width / 2.0)];
                    // COUT2("cone.y + cone.height/2:",cone.y + cone.height/2);
                
#ifdef CAR_DEBUG
                    // 边线显示
                    for (int i = 0; i < rpts0s_num; i++) 
                        cv::circle(ff, cv::Point(rpts0s[i][0], rpts0s[i][1]), 1, Scalar(255, 0, 0));
                    // 边线显示
                    for (int i = 0; i < rpts1s_num; i++) 
                        cv::circle(ff, cv::Point(rpts1s[i][0], rpts1s[i][1]), 1, Scalar(0, 0, 255));
#endif
                    double ldx = 0;
                    int lindex = 0;
                    for (int i = 0; i < MIN(rpts0s_num, 80); i++) {
#ifdef CAR_DEBUG
                        // 锥桶识别中心坐标
                        cv::circle(ff, cv::Point(cone_poi_x, cone_poi_y), 5, Scalar(255, 0, 0));
#endif
                        if((int)cone_poi_y == (int)rpts0s[i][1] || (int)cone_poi_y - 1 == (int)rpts0s[i][1] || (int)cone_poi_y + 1 == (int)rpts0s[i][1]) {
#ifdef CAR_DEBUG
                            // 距离测算点
                            cv::line(ff, cv::Point(0, rpts0s[i][1]), cv::Point(rpts0s[i][0], rpts0s[i][1]), Scalar(255, 112, 132));
#endif
                            ldx = cone_poi_x - rpts0s[i][0];
                            lindex = i;
                            break;
                        }
                    }

                    double rdx = 0;
                    int rindex = 0;

                    for (int i = 0; i < MIN(rpts1s_num, 80); i++) {
#ifdef CAR_DEBUG
                        // 锥桶识别中心坐标
                        cv::circle(ff, cv::Point(cone_poi_x, cone_poi_y), 5, Scalar(0, 0, 255));
#endif
                        if((int)cone_poi_y == (int)rpts1s[i][1] || (int)cone_poi_y - 1 == (int)rpts1s[i][1] || (int)cone_poi_y + 1 == (int)rpts1s[i][1]) {
#ifdef CAR_DEBUG      
                            // 距离测算点
                            cv::line(ff, cv::Point(rpts1s[i][0], rpts1s[i][1]),cv::Point(COLSIMAGE - 1, rpts1s[i][1]), Scalar(139, 236, 255));
#endif
                            rdx = rpts1s[i][0] - cone_poi_x;
                            rindex = i;
                            break;
                        }
                    }
                    // COUT2(ldx, rdx);
                    // 左侧锥桶
                    if(ldx > 0 && ldx < rdx) {
                        last_is_cone = true;
                        if(L_count++ >= 2) {
                            last_turn_left = false;
                            L_count = R_count = 0;
                            state = 10;
                            // if(!blocks.empty() && cones.size() == 1) {
                            //     COUT2("CONE->BLOCK : [L] 障碍物靠左",ldx);
                            //     mid_x = block_mid_L;
                            // }else{
                                COUT2("[L] 障碍物靠左",ldx);
                                mid_x = mid_L;
                            // }
                            buzzer = 1;
                            track_type = TRACK_RIGHT;
                            danger_count = 0;
                        }
                    }
                    // 右侧锥桶
                    else if(rdx > 0 && rdx < ldx) {
                        last_is_cone = true;
                        if(R_count++ >= 2) {
                            last_turn_left = true;
                            L_count = R_count = 0;
                            state = 20;
                            // if(!blocks.empty() && cones.size() == 1) {
                            //     COUT2("CONE->BLOCK : [R] 障碍物靠右",rdx);
                            //     mid_x = block_mid_R;
                            // }else{
                                COUT2("[R] 障碍物靠右",rdx);
                                mid_x = mid_R;
                            // }
                            buzzer = 1;
                            track_type = TRACK_LEFT;
                            danger_count = 0;
                        }
                    }else{
                        L_count = R_count = 0;
                        if(!blocks.empty()) goto block;
                        COUT1("!coneempty NONE LDX RDX");
                        if(state != 1) {
                            if (danger_count++ >= 2) {
                                if(last_turn_left) {
                                    L_count = R_count = 0;
                                    state = 10;
                                    mid_x = mid_L;
                                    COUT2("turn R",ldx);
                                    buzzer = 1;
                                    track_type = TRACK_RIGHT;
                                    danger_count = 0;
                                }else{
                                    L_count = R_count = 0;
                                    state = 20;
                                    mid_x = mid_R;
                                    COUT2("turn L",rdx);
                                    buzzer = 1;
                                    track_type = TRACK_LEFT;
                                    danger_count = 0;
                                }
                                danger_count = 0;
                                // buzzer = 0;
                                // mid_x = MIDX_MID;
                                // state = 0;
                                // AI_ENABLE = DANGER_ENABLE = false;
                                // danger_type = DangerType::NONE;
                            }
                        }
                    }
                }else{
                    block:;
                    if(!blocks.empty()) {
                        int areaMax = 0; // 框面积
                        int index = 0;   // 目标序号
                        for (int i = 0; i < blocks.size(); i++) {
                            int area = blocks[i].width * blocks[i].height;
                            if (area >= areaMax) {
                                index = i;
                                areaMax = area;
                            }
                        }

                        block = blocks[index];
                        enable = true; // 场景检测使能标志

                        // 障碍物方向判定（左/右）
                        double block_poi_x = mapx[(int)(block.y + block.height / 2.0)][(int)(block.x + block.width/2.0)];
                        double block_poi_y = mapy[(int)(block.y + block.height / 2.0)][(int)(block.x + block.width/2.0)];
                        // COUT2("block.y + block.height/2:",block.y + block.height/2);
                    
    #ifdef CAR_DEBUG
                        // 边线显示
                        for (int i = 0; i < rpts0s_num; i++) 
                            cv::circle(ff, cv::Point(rpts0s[i][0], rpts0s[i][1]), 1, Scalar(255, 0, 0));
                        // 边线显示
                        for (int i = 0; i < rpts1s_num; i++) 
                            cv::circle(ff, cv::Point(rpts1s[i][0], rpts1s[i][1]), 1, Scalar(0, 0, 255));
    #endif
                        double ldx = 10000;
                        int lindex = 0;
                        for (int i = 0; i < MIN(rpts0s_num, 80); i++) {
    #ifdef CAR_DEBUG
                            // 锥桶识别中心坐标
                            cv::circle(ff, cv::Point(block_poi_x, block_poi_y), 5, Scalar(255, 0, 0));
    #endif
                            if((int)block_poi_y == (int)rpts0s[i][1] || (int)block_poi_y - 1 == (int)rpts0s[i][1] || (int)block_poi_y + 1 == (int)rpts0s[i][1]) {
    #ifdef CAR_DEBUG
                                // 距离测算点
                                cv::line(ff, cv::Point(0, rpts0s[i][1]), cv::Point(rpts0s[i][0], rpts0s[i][1]), Scalar(255, 112, 132));
    #endif
                                ldx = block_poi_x - rpts0s[i][0];
                                lindex = i;
                                break;
                            }
                        }

                        double rdx = 10000;
                        int rindex = 0;

                        for (int i = 0; i < MIN(rpts1s_num, 80); i++) {
    #ifdef CAR_DEBUG
                            // 锥桶识别中心坐标
                            cv::circle(ff, cv::Point(block_poi_x, block_poi_y), 5, Scalar(0, 0, 255));
    #endif
                            if((int)block_poi_y == (int)rpts1s[i][1] || (int)block_poi_y - 1 == (int)rpts1s[i][1] || (int)block_poi_y + 1 == (int)rpts1s[i][1]) {
    #ifdef CAR_DEBUG      
                                // 距离测算点
                                cv::line(ff, cv::Point(rpts1s[i][0], rpts1s[i][1]),cv::Point(COLSIMAGE - 1, rpts1s[i][1]), Scalar(139, 236, 255));
    #endif
                                rdx = rpts1s[i][0] - block_poi_x;
                                rindex = i;
                                break;
                            }
                        }
                        COUT1("BLOCK::::::");
                        COUT2(ldx, rdx);
                        // 左侧锥桶
                        if(abs(ldx) < abs(rdx)) {
                            if(L_count++ >= 2) {
                                L_count = R_count = 0;
                                state = 10;
                                mid_x = block_mid_L;
                                COUT2("[L] block靠左",ldx);
                                buzzer = 1;
                                track_type = TRACK_RIGHT;
                                danger_count = 0;
                            }
                        }
                        // 右侧锥桶
                        else if(abs(rdx) < abs(ldx)) {
                            if(R_count++ >= 2) {
                                L_count = R_count = 0;
                                state = 20;
                                mid_x = block_mid_R;
                                COUT2("[R] block靠右",rdx);
                                buzzer = 1;
                                track_type = TRACK_LEFT;
                                danger_count = 0;
                            }
                        }else{
                            L_count = R_count = 0;
                            COUT1("!block  ldx == rdx");
                            if(state != 1) {
                                if (danger_count++ >= 10) {
                                    buzzer = 0;
                                    mid_x = MIDX_MID;
                                    state = 0;
                                    AI_ENABLE = DANGER_ENABLE = false;
                                    danger_type = DangerType::NONE;
                                }
                            }
                        }

                        if(last_is_cone) {
                            last_is_cone = false;
                            if(last_turn_left) {
                                L_count = R_count = 0;
                                state = 10;
                                mid_x = block_mid_L;
                                COUT1("block turn R");
                                buzzer = 1;
                                track_type = TRACK_RIGHT;
                                danger_count = 0;
                            }else{
                                L_count = R_count = 0;
                                state = 20;
                                mid_x = block_mid_R;
                                COUT1("block turn L");
                                buzzer = 1;
                                track_type = TRACK_LEFT;
                                danger_count = 0;
                            }
                        }
                    }
                    else if(!have_bomb && resultsObs.empty()) {
                        COUT1("!have_bomb && resultsObs.empty()");
                        if(SPEED_DANGER < 20) {
                            if(danger_count++ >= 10) {
                                buzzer = 0;
                                mid_x = MIDX_MID;
                                state = 0;
                                AI_ENABLE = DANGER_ENABLE = false;
                                danger_type = DangerType::NONE;
                            }
                        }else{
                            if(danger_count++ >= 3) {
                                buzzer = 0;
                                mid_x = MIDX_MID;
                                state = 0;
                                AI_ENABLE = DANGER_ENABLE = false;
                                danger_type = DangerType::NONE;
                            }
                        }
                    }

                    if(have_bomb || !resultsObs.empty()) {
                        danger_count = 0;
                    }
                }
#ifdef CAR_SHOW
                imshow("fixff", ff);
#endif
#ifdef CAR_SHOW
                imshow("danger",danger_frame);
#endif
                break;
            }
        }

        
        return enable;
    }

    /**
     * @brief 图像绘制禁行区识别结果
     *
     * @param img 需要叠加显示的图像
     */
    void drawImage(Mat &img)
    {
        if (enable)
        {
            // putText(img, "[2] DANGER - ENABLE", Point(COLSIMAGE / 2 - 30, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
            // cv::Rect rect(resultObs.x, resultObs.y, resultObs.width, resultObs.height);
            // cv::rectangle(img, rect, cv::Scalar(0, 0, 255), 1);
        }
    }

private:
    int L_count = 0;
    int R_count = 0;
    bool enable = false;    // 场景检测使能标志
    int danger_count = 0;   // 场次
    PredictResult cone; // 避障目标锥桶
    PredictResult block; // 避障目标路障

};
