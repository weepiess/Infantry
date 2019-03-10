////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC ROBOMASTER2018      Model Code for robot
///ALL RIGHTS RESERVED
///@file:control_model.h
///@brief: robot 控制模型，包含对所有应用的管理，创建应用，并改变应用，以及监听操作手的指令
///，并处理。
///@vesion 1.0
///@author: gezp
///@email: 1350824033@qq.com
///@date: 18-9-4
///修订历史：
////////////////////////////////////////////////////////////////////////////////


#ifndef RMDEMO_CONTROL_MODEL_H
#define RMDEMO_CONTROL_MODEL_H
#include "opencv2/opencv.hpp"
#include "robot_model.h"
#include "auto_aim.h"
#include "aim_assistant.h"
#include <future>
#include "basic_tool.h"
#include "mind_vision.h"
#include "autoptz.h"
#include "datathread.h"

class ControlModel{
public:
    ControlModel();
    ~ControlModel();

public:
    void init(RobotModel* pRobotModel);
    //串口监听数据处理函数接口
    void serialListenDataProcess(SerialPacket recvPacket);
    void processFSM();

private:
    //机器人临时模式变量
    cv::FileStorage file;
    RobotMode mSetMode;
    BasicTool basic_tool;
    AutoAim autoAim;
    AutoPTZ autoptz;
  //  Aim_assistant aim_assist;
    vector<double> result;
    int armor_id=-1;
private:
    RobotModel* pRobotModel;
    //相关临时变量
    MindVision* cap;
    SerialInterface* interface;
        //是否需要确认装甲板
    bool need_check=true;
    std::future<int> tmp;
    NYdector mthread;
    Aim_assistant aim_assistant;
private:
    void Aim(bool is_shoot_control=true);
    void AutoPTZControl();
    void playerAim();

};

#endif //RMDEMO_CONTROL_MODEL_H
