////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC ROBOMASTER2019      Model Code for robot
///ALL RIGHTS RESERVED
///@file:aim_predict.h
///@brief: 自瞄运动预测模型,使用卡尔曼滤波算法进行运动预测.
/// 
///@vesion 1.0
///@author: pc
///@email: 694977655@qq.com
///@date: 18-12-9
///修订历史：
////////////////////////////////////////////////////////////////////////////////
#ifndef AIM_PREDICT_PITCH_H
#define AIM_PREDICT_PITCH_H
#include "kalman_filter_by_opencv.h"
//#include "Kalman_CSM.h"
#include "opencv2/opencv.hpp"
#include <numeric>

class Aim_predict_pitch{
    public:
        Aim_predict_pitch();
        ~Aim_predict_pitch();
    public: 
        //预测模型初始化
        void modelInit();
        
        //根据时间 重新调整卡尔曼statepost
        void resetKfStatepost(Mat statepost);

        /** 预测函数
        *  @param: Mat measurement, 测量矩阵
        *  @param: float dt, 采样周期
        *  @return: Mat 预测矩阵
        */
        Mat predict(cv::Mat measurement,float dt);

        /** 弹丸激发策略函数
         *  @param: Point3d tvec, PNP解算后出的云台转角
         *  @param: float angel_velocity, 当前云台角速度
         *  @param: float predict_angle, 卡尔曼预测的提前量
         */
        bool shootLogic(float initYaw, float angel_velocity, float predict_angle);
        
        //丢失目标后清空vector容器,准备下一次数据
        void clear();
    private:    
        Kalman_filter kalman_fileter_;
        vector<float> yaw_;
        vector<float> angle_velocity_;
        vector<float> predict_angle_;




};
#endif
