////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC ROBOMASTER2018      Thread Code for robot
///ALL RIGHTS RESERVED
///@file:serial_listen_thread.cpp
///@brief: 串口监听线程控制。除了少量的数据校验，不包括数据处理，
/// 数据处理通过机器人控制模型（ControlModel）中的接口实现。
///@vesion 1.0
///@author: gezp
///@email: 1350824033@qq.com
///@date: 18-3-4
///修订历史：
////////////////////////////////////////////////////////////////////////////////
#include "serial_listen_thread.h"
SerialListenThread::SerialListenThread() {
    exit_flag_= false;
}
SerialListenThread::~SerialListenThread(){

}
void SerialListenThread::init(RobotModel *robot_model,ControlModel *control_model){
    robot_model_=robot_model;
    control_model_=control_model;
};

void SerialListenThread::run() {
    SerialPacket recvPacket;
    while(!exit_flag_){
        //是否需要考虑对监听数据构造队列，因为处理数据函数需要一定时间。
        if(robot_model_->getpSerialInterface()->dataRecv(recvPacket)==0){
                //串口监听数据处理
                control_model_->serialListenDataProcess(recvPacket);
        }

    }
}