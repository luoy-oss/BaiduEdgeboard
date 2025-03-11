#include "../code/camera_param.h"
#include "../src/main.h"
#include <vector>
#include <numeric>
#include <cmath>

#include "../code/common.h"
#include "../code/detection.hpp" // Ai模型预测

using namespace std;
using namespace cv;


int rescue_state = 0;	// 串口救援区状态(uart.h 引用 : (extern) 0 NONE 10 左入 20 右入 30可以回归原始寻线 
bool RESCUE_ENABLE = false;
extern float mid_x;
extern float mid_y;

class Rescue {
public:
    int counterImmunity = 0;      // 屏蔽计数器
    int TopConeY = 44;     // 判入行
    
    int OUTConeWidth = 240;     // 左右锥桶判出距离

    int OUTConeDOWNY = 160;     // 最下侧锥桶判出行
    int OUTConeUPY = 115;       // 最上侧锥桶判出行

    enum RescueType{
        NONE = 0,
        BEGIN,
        LEFT_IN,
        RIGHT_IN,
        OUT,
        FINISH,
        NUM
    };
    RescueType rescue_type = RescueType::NONE;
    const char* rescue_type_name[RescueType::NUM] = {
        "RESCUE_NONE",
        "RESCUE_BEGIN",
        "RESCUE_LEFT_IN","RESCUE_RIGHT_IN",
        "RESCUE_OUT",
        "RESCUE_FINISH"
    };

    void reset() {
        rescue_state = 0;
        topCone = PredictResult();      //最上侧锥桶，用于判入
        leftCone = PredictResult();     //最左侧锥桶，用于判出
        rightCone = PredictResult();    //最右侧锥桶，用于判出            
        enable = false;
        again = false;
        counterImmunity = 0;
        rescue_count = 0;
        left_count = 0;
        right_count = 0;
        rescue_type = RescueType::NONE;
        RESCUE_ENABLE = false;
    }

    /**
     * @brief 救援区AI识别与路径规划处理
     *
     * @param track 赛道识别结果
     * @param predict AI检测结果
     * @return true
     * @return false
     */
    bool process(vector<PredictResult> predict)
    {
        // vector<PredictResult> resultsObs; // 锥桶AI检测数据
        // resultsObs.clear();
        int cone_count = 0;
        switch(rescue_type) {
            case RescueType::NONE:
                if(/*(counterImmunity > 500 && again)*/!again){
                    for (int i = 0; i < predict.size(); i++){
                        if(predict[i].type == LABEL_EVIL || predict[i].type == LABEL_THIEF && predict[i].y > 15) {
                            right_count++;
                        }else if(predict[i].type == LABEL_PATIENT || predict[i].type == LABEL_TUMBLE && predict[i].y > 15) {
                            left_count++;
                        }else if(predict[i].type == LABEL_CONE) {
                            cone_count++;
                        }
                    }
                    COUT2("cone_count:\t", cone_count);

                    if(left_count || right_count){
                        if(rescue_count++ >= 20) {
                            cone_count = 0;
                            rescue_type = RescueType::NONE;
                            mid_x = MIDX_MID;
                            right_count = 0;
                            rescue_count = 0;
                        }
                        if(cone_count >= 8) {
                            RESCUE_ENABLE = true;
                            if (left_count >= 2 && left_count > right_count) {
                                cone_count = 0;
                                rescue_type = RescueType::BEGIN; // 使能
                                mid_x = MIDX_MID;
                                right_count = 0;
                                rescue_count = 0;
                            } else if (right_count >= 2 && right_count > left_count) {
                                cone_count = 0;
                                mid_x = MIDX_MID;
                                rescue_type = RescueType::BEGIN; // 使能
                                left_count = 0;
                                rescue_count = 0;
                            }
                        }

                    }
                }else counterImmunity++;

                break;
            case RescueType::BEGIN:
                RESCUE_ENABLE = true;
                buzzer = 1;
                for (int i = 0; i < predict.size(); i++) {
                    // AI标志距离计算
                    if ((predict[i].type == LABEL_CONE)) {
                        if(i == 0) {
                            topCone = predict[i];
                        }
                        else {
                            if(predict[i].y + predict[i].height/2 < topCone.y + topCone.height/2) {
                                topCone = predict[i];
                            }
                        }
                    }
                }
                // COUT2("topConeY: ",topCone.y + topCone.height/2);
                // COUT2(leftCone.x + leftCone.width/2, leftCone.y + leftCone.height/2);
                // COUT2(rightCone.x + rightCone.width/2, rightCone.y + rightCone.height/2);

                // static bool is_label = false;
                // for (int i = 0; i < predict.size(); i++){
                //     if( predict[i].type == LABEL_EVIL || predict[i].type == LABEL_THIEF || 
                //         predict[i].type == LABEL_PATIENT || predict[i].type == LABEL_TUMBLE) {
                //         is_label = true;
                //         break;
                //     }
                // }
                if(topCone.y + topCone.height/2 >= TopConeY) {
                    if(left_count){
                        rescue_count = 0;
                        rescue_state = 10; //左入
                        rescue_type = RescueType::LEFT_IN;
                    }else if(right_count) {
                        rescue_count = 0;
                        rescue_state = 20; //右入
                        rescue_type = RescueType::RIGHT_IN;
                    }
                }
                if(rescue_count++ >= 40) {
                    mid_x = MIDX_MID;
                    RESCUE_ENABLE = false;
                    rescue_count = 0;
                    rescue_state = 0;
                    rescue_type = RescueType::NONE;
                }
                break;
            case RescueType::LEFT_IN:
            case RescueType::RIGHT_IN:
                buzzer = 0;
                // COUT2(rpts0s_num, rpts1s_num);
                // 根据白色多边形填充 沿多边形外沿循迹 点数51  Lr Rr半径12 13
                if(rpts0s_num < 60 && rpts1s_num < 60 && Lradius < 20 && Rradius < 20) {
                    rescue_count = 0;
                    enable = true;
                }

                if(enable) {
                    // COUT1("ENABLE!");
                    if(rpts0s_num > 60 && rpts1s_num > 60 && Lradius > 100 && Rradius > 100) {
                        rescue_state = 30;
                        buzzer = 1;
                        // if(rescue_count++ >= 2) {
                            rescue_count = 0;
                            rescue_type = RescueType::OUT;
                        // }
                    }
                }
                
                
                break;
            case RescueType::OUT:
                rescue_count = 0;
                enable = false;
                counterImmunity = 0;
                again = true;
                rescue_state = 0;
                RESCUE_ENABLE = false;
                rescue_type = RescueType::NONE;
                break;
        }

        // if((rightCone.x + rightCone.width/2) - (leftCone.x + leftCone.width/2) >= OUTConeWidth /*&&
        //     ((leftCone.y + leftCone.height/2 >= OUTConeDOWNY && rightCone.y + rightCone.height/2 >= OUTConeUPY ) ||
        //     (leftCone.y + leftCone.height/2 >= OUTConeUPY && rightCone.y + rightCone.height/2 >= OUTConeDOWNY))*/) {
        //     // rescue_type = RescueType::OUT;
        // }

        return enable;
    }

    /**
     * @brief 图像绘制救援区识别结果
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
    bool again = false; // 第二次进入救援区标志
    bool enable = false;    // 场景检测使能标志
    int rescue_count = 0;   // 场次
    int left_count = 0;     // 判左入场次（平民伤员）
    int right_count = 0;    // 判右入场次（危险人物）
    PredictResult topCone;      //最上侧锥桶，用于判入
    PredictResult leftCone;     //最左侧锥桶，用于判出
    PredictResult rightCone;    //最右侧锥桶，用于判出
};
