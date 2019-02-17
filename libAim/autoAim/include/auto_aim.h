#ifndef AUTO_AIM_H
#define AUTO_AIM_H

#include <opencv2/opencv.hpp>
#include "base_aim.h"
#include "time.h"
#include "aim_predict.h"
#include "usb_capture_with_opencv.h"
#include "aim_assistant.h"
//#include "aim_assistant.h"
using namespace cv;
using namespace std;

class AutoAim: public BaseAim{
public:
    AutoAim();
    AutoAim(int width, int height);
    ~AutoAim();

public:
    bool setImage(Mat &src);
    void findLamp_rect(vector<RotatedRect> &pre_armor_lamps); //搜索所有可能灯条
    void match_lamps(vector<RotatedRect> &pre_armor_lamps, vector<RotatedRect> &real_armor_lamps); //匹配灯条
    void select_armor(vector<RotatedRect> real_armor_lamps); //锁定装甲板
    //aim()的形式不固定，但是返回值必须是AimResult类型
    AimResult aim(Mat &src, float currPitch, float currYaw, Point2f &pitYaw,int is_predict,bool &if_shoot,float time_delay);
    void set_parameters(int angle,int inside_angle, int height, int width);
    Rect armor;


private:

    void resetROI();


private:
    int param_diff_angle;
    int param_inside_angle;
    int param_diff_height;
    int param_diff_width;
    int count=0;
    float lastPitch=0;
    float lastYaw=0;
    Point3f last_tvec;
    int resizeCount;
    RotatedRect best_lamps[2];
    Mat measurement = Mat::zeros(1, 1, CV_32F);
    float dt;
    Point bestCenter;
    Mat mask;
    bool is_check_armor=true;
    Rect rectROI;
    Aim_predict aim_predict;
    Aim_assistant aim_assistant;
};                                                                                                      

#endif
