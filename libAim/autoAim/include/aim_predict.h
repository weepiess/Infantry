////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC ROBOMASTER2019      Model Code for robot
///ALL RIGHTS RESERVED
///@file:aim_predict.h
///@brief: 自瞄预测部分统一接口
/// 
///@vesion 1.0
///@author: pc
///@email: 694977655@qq.com
///@date: 18-11-4
///修订历史：
////////////////////////////////////////////////////////////////////////////////
#ifndef AIM_PREDICT_H
#define AIM_PREDICT_H
#include "kalman_filter_by_opencv.h"
#include "opencv2/opencv.hpp"

class Aim_predict{
    public:
        Aim_predict();
        ~Aim_predict();
    public: 
        //预测模型初始化
        void model_init();
        
        //重新调整卡尔曼statepost,主要用于对新目标的statepost更新
        void reset_kf_statepost(Mat statepost);

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
        bool shoot_logic(float initYaw, float angel_velocity, float predict_angle);
        
        //丢失目标后清空vector容器,准备下一次数据
        void clear();
    private:    
        Kalman_filter mKf;
        vector<float> mYaw;
        vector<float> mAngle_Velocity;
        vector<float> mPredict_Angle;




};
#endif
