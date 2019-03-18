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
#include "recognize_wiggle.h"
using namespace cv;
using namespace std;

class AutoAim: public BaseAim{
public:
    AutoAim();
    ~AutoAim();

public:
    void init(Aim_assistant* checker);
    bool setImage(Mat &src);
    void findLampRect(vector<RotatedRect> &pre_armor_lamps); //搜索所有可能灯条
    void matchLamps(vector<RotatedRect> &pre_armor_lamps, vector<RotatedRect> &real_armor_lamps); //匹配灯条
    void selectArmor(vector<RotatedRect> real_armor_lamps); //锁定装甲板
    void selectArmorH(vector<RotatedRect> real_armor_lamps);//操作手选定装甲板函数
    //aim()的形式不固定，但是返回值必须是AimResult类型
    AimResult aim(Mat &src, float curr_pitch, float curr_yaw, Point2f &pitch_yaw,int is_predict,bool &if_shoot,float time_delay);
    Rect armor;

private:
    
    float dataDealer(float new_num);
    void resetRoi();


public:
    uchar g_condition = 1;  // 扭腰击打模式

private:
    Mat image_;
    Mat measurement_ = Mat::zeros(1, 1, CV_32F);
    Mat mask_;
    Rect rect_roi_;
    RotatedRect best_lamps_[2];
    Point best_center_;

private:
    int resize_count_;
    int count_=0;
    int num_ = 0;      //一个过渡值，对于同方向变化量大的数据，num_x越大 
    int old_inverse_flags_ = 0;   //表示第n-2个数据到第n-1个数据的变化趋势，加为1，减为0 
    int new_inverse_flags_ = 0;   //表示第n-1个数据到第n个数据的变化趋势，加为1，减为0
    float old_num_ = 0;     //第n-1次的答案 
    int threshold_min_ = 0;
    int threshold_max_ = 11;
    float k_ =0.6;
    float k_add_;
    int path_number_ = 0 ;
    int count_hero_=0;  //锁定近处步兵计数器
    bool hero_lock_ = false;  //取消锁定
    bool infantry_lock_ = false;  //锁定近处步兵
    bool is_right_ = false;
    bool special_condition_ = false;

private:
    RecognizeWiggle recognize_wiggle_;
    Aim_predict aim_predict_;
    Aim_assistant* id_checker_;

};                                                                                                      
#endif

