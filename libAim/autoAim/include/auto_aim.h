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
#include "autotrn.h"
using namespace cv;
using namespace std;

class AutoAim: public BaseAim{
public:
    AutoAim();
    ~AutoAim();

public:
// friend class AutoTRN;

public:
    void init(Aim_assistant* checker);
    bool setImage(Mat &src);
    void findLamp_rect(vector<RotatedRect> &pre_armor_lamps); //搜索所有可能灯条
    void match_lamps(vector<RotatedRect> &pre_armor_lamps, vector<RotatedRect> &real_armor_lamps); //匹配灯条
    void select_armor(vector<RotatedRect> real_armor_lamps); //锁定装甲板
    void selectArmorH(vector<RotatedRect> real_armor_lamps);//操作手选定装甲板函数
    //aim()的形式不固定，但是返回值必须是AimResult类型
    AimResult aim(Mat &src, float curr_pitch, float curr_yaw, Point2f &pitch_yaw,int is_predict,bool &if_shoot,float time_delay);
    void set_parameters(int angle,int inside_angle, int height, int width);
    Rect armor;

private:
    
    float dataDealer(float new_num);
    void resetROI();


public:
    uchar condition = 1;  // 扭腰击打模式

private:
    Mat image;
    Mat measurement = Mat::zeros(1, 1, CV_32F);
    Mat mask;
    Rect rectROI;
    Point3f last_tvec;
    RotatedRect best_lamps[2];
    Point bestCenter;

private:
    float dt;
    int resizeCount;
    int param_diff_angle;
    int param_inside_angle;
    int param_diff_height;
    int param_diff_width;
    int count=0;
    int num = 0;      //一个过渡值，对于同方向变化量大的数据，num_x越大 
    int old_inverse_flags = 0;   //表示第n-2个数据到第n-1个数据的变化趋势，加为1，减为0 
    int new_inverse_flags = 0;   //表示第n-1个数据到第n个数据的变化趋势，加为1，减为0
    float old_num = 0;     //第n-1次的答案 
    int threshold_min = 0;
    int threshold_max = 11;
    float k =0.6;
    float k_add;
    int c = 0 ;
    int count_hero=0;  //锁定近处步兵计数器
    bool hero_lock = false;  //取消锁定
    bool infantry_lock = false;  //锁定近处步兵
    bool is_right = false;
    bool special_condition = false;
    bool is_check_armor=true;

private:
    AutoTRN autotnr;
    Aim_predict aim_predict;
    Aim_assistant* id_checker;

};                                                                                                      
#endif

