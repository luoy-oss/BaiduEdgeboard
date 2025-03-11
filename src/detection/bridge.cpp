#include "../code/camera_param.h"
#include "../src/main.h"
#include <vector>
#include <numeric>
#include <cmath>

#include "../code/common.h"
#include "../code/detection.hpp" // Ai模型预测


using namespace std;
using namespace cv;

bool BRIDGE_ENABLE = false;

/**
 * @brief 桥AI识别与路径规划类
 *
 */
class Bridge
{

public:
    int state = 0;
    int bridge_y_top = 86;

    enum BridgeType{
        NONE = 0,
        RUN,
        END,
        NUM
    };

    BridgeType bridge_type = BridgeType::NONE;
    const char* bridge_type_name[BridgeType::NUM] = {
        "BRIDGE_NONE",
        "BRIDGE_RUN",
        "BRIDGE_END",
    };
    void _init_() { }
    void reset() {
        bridge_type = BridgeType::NONE;
    }

    bool process(vector<PredictResult> predict) {
        enable = false; // 场景检测使能标志
        switch(bridge_type) {
            case BridgeType::NONE: {
                for (int i = 0; i < predict.size(); i++){
                    if(predict[i].type == LABEL_BRIDGE && predict[i].y + predict[i].height/2 > bridge_y_top) {
                        COUT2("LABEL_BRIDGE", predict[i].y);
                        bridge_count++;
                        state = 1;
                    }
                }

                if(bridge_count >= 2) {
                    none_count = 0;
                    bridge_count = 0;
                    BRIDGE_ENABLE = true;
                    bridge_type = BridgeType::RUN;
                    state = 1;
                }

                if(bridge_count == 1 && none_count++ >= 20) {
                    COUT1("bridge_count == 1 && none_count++ >= 20");
                    none_count = 0;
                    bridge_count = 0;
                    state = 0;
                    bridge_type = BridgeType::NONE;
                }
                break;
            }
            case BridgeType::RUN: {
                BRIDGE_ENABLE = true;
                vector<PredictResult> bridges;
                for (int i = 0; i < predict.size(); i++) {
                    if (predict[i].type == LABEL_BRIDGE)  {
                        bridges.push_back(predict[i]);
                        break;
                    }
                }

                if(!bridges.empty()) {
                    state = 10;
                    bridge_count = 0;
                    COUT1("! bridges.empty() >>> waiting...");
                }else{
                    if(bridge_count++ > 20) {
                        COUT1(">>>> bridge  END");
                        COUT1(">>>> bridge  END");
                        COUT1(">>>> bridge  END");
                        buzzer = 0;
                        state = 0;
                        bridge_count = 0;
                        BRIDGE_ENABLE = false;
                        bridge_type = BridgeType::NONE;
                    }
                }
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
    bool enable = false;    // 场景检测使能标志
    int bridge_count = 0;   // 场次
    int none_count = 0;

};
