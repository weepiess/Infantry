////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC ROBOMASTER2019      Model Code for robot
///ALL RIGHTS RESERVED
///@file:aim_assisttant.h
///@brief: 自瞄识别助手,用于装甲板识别验证及其id识别
/// 
///@vesion 1.0
///@author: pc
///@email: 694977655@qq.com
///@date: 18-12-9
///修订历史：
////////////////////////////////////////////////////////////////////////////////
#ifndef AUTO_AIM_ASSISTANT_H
#define AUTO_AIM_ASSISTANT_H
#include <iostream>
#include "opencv2/opencv.hpp"
#include "tensorflow/core/public/session.h"
#include "tensorflow/core/platform/env.h"
#include "tensorflow/cc/saved_model/loader.h"
#include "tensorflow/core/framework/graph.pb.h"
#include "tensorflow/core/protobuf/meta_graph.pb.h"
#include  "tensorflow/cc/saved_model/tag_constants.h"
using namespace tensorflow;
using namespace std;
class Aim_assistant{
    public:
        Aim_assistant();
        ~Aim_assistant();
    public:
        /** 初始化函数
        *   CNN模型初始化
        *  @param:  string model_path :保存的CNN .pb模型位置
        *  @return: int :错误号，1代表无错误，-１代表发生错误。
        */
        int init(string model_path);

        /** 初始化函数
        *   装甲板id识别函数
        *  @param:  cv::Mat src :输入装甲板图片
        *  @return: int :装甲板id数字 1-5  返回-1 识别错误
        */
        int check_armor(cv::Mat src);
        
    private:
        
        void TensorInit(Session* session,cv::Mat& img);
    private:
        Session *session;
        GraphDef graph_def;

};
#endif