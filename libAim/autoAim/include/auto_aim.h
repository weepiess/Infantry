#ifndef AUTO_AIM_H
#define AUTO_AIM_H
#define LIKELY(x) __builtin_expect(!!(x),1)

#include <opencv2/opencv.hpp>
#include "base_aim.h"
#include "time.h"
#include "aim_predict.h"
#include "usb_capture_with_opencv.h"
#include "aim_assistant.h"
#include "queue"
#include <list>
#include "Fft.h"
using namespace cv;
using namespace std;

class AutoAim: public BaseAim{
public:
    AutoAim();
    ~AutoAim();

public:
    void init(Aim_assistant* checker);
    bool setImage(Mat &src);
    void findLamp_rect(vector<RotatedRect> &pre_armor_lamps); //搜索所有可能灯条
    void match_lamps(vector<RotatedRect> &pre_armor_lamps, vector<RotatedRect> &real_armor_lamps); //匹配灯条
    void select_armor(vector<RotatedRect> real_armor_lamps); //锁定装甲板
    void selectArmorH(vector<RotatedRect> real_armor_lamps);//操作手选定装甲板函数
    //aim()的形式不固定，但是返回值必须是AimResult类型
    AimResult aim(Mat &src, float currPitch, float currYaw, Point2f &pitYaw,int is_predict,bool &if_shoot,float time_delay);
    void set_parameters(int angle,int inside_angle, int height, int width);
    Rect armor;
    void datajudger();
private:
    //Thread function_thread;
    FFt fft;
private:
    
    float datadealer(float new_num);
    void resetROI();


private:
    bool special_condition = false;
    Mat image;
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
    Mat source_image;
    Mat watchwin;
    bool is_check_armor=true;
    Rect rectROI;
    Aim_predict aim_predict;
    Aim_assistant* id_checker;
    float ratio1_max;
    float ratio1_min;
    float ratio2_max;
    float ratio2_min;
    float max=-1;
    int num = 0;      //一个过渡值，对于同方向变化量大的数据，num_x越大 
    int old_flags = 0;   //表示第n-2个数据到第n-1个数据的变化趋势，加为1，减为0 
    int new_flags = 0;   //表示第n-1个数据到第n个数据的变化趋势，加为1，减为0
    float old_num = 0;     //第n-1次的答案 
    int Threshold_min = 8;
    int Threshold_max = 20;
    list<float> numbase;
    double result[4095 * 2] = { 0 };
    float frequency=50;
    int badnum = 0;
    bool badcondition = false;
    int key = 0;
    float new_number;
    float old_number = 0;
    float k =0.2;
    int c = 0 ;
};                                                                                                      
#endif


// frame_speed: 1
// gamma: 32
// exposure_time: 1100
// contrast: 92
// ae_state: 0
// resolution_width: 1280
// resolution_height: 720
