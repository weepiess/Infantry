#ifndef MARK_AIM_H
#define MARK_AIM_H
#include "base_aim.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <algorithm>

using namespace cv;
using namespace std;

class MarkAim: public BaseAim{
    public:
        MarkAim();
        ~MarkAim();

    public:
        void init();//初始化
        void setImage(Mat &input, Mat &Output);//图像预处理
        int locateArmor(Mat src,Mat Mask,RotatedRect real_armor,vector<Point2f> &armor_points);//确定装甲板坐标
        BaseAim::AimResult Aim(RotatedRect real_armor,vector<Point2f> armor_points,float current_pitch,float current_yaw,Point2f &ctrl_angel);//瞄准与控制逻辑;

    private:
        int find_R(vector<Rect> rect, Mat img);
        int findArmor(vector<RotatedRect> rect);
        int findUndecidedArea(vector<Rect> rect);
        int findUndecidedArea(vector<RotatedRect> rect);
        float distance(Point2f point1,Point2d point2);
        float calAngle(Point2f armor_center,Point2d center);
        Point2d calLocation(float angle,float radius,Point2d center);
    private:
        Point2d center;
        float radius = 0;
        int count = 1;
        int center_box_index = -1;//大符中心包围盒序号
        
};

#endif