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
        int init(string model_path);
        int check_armor(cv::Mat src);
        
    private:
        void TensorInit(Session* session,cv::Mat& img);
    private:
        Session *session;
        GraphDef graph_def;

};
#endif