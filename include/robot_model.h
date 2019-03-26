////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC ROBOMASTER2018      Model Code for robot
///ALL RIGHTS RESERVED
///@file:robot_model.h
///@brief: 机器人数据资源模型，包含机器人的所有硬件资源的统一管理，
/// 以及状态数据的统一管理，并提供机器人控制基本接口。
///@vesion 1.0
///@author: gezp
///@email: 1350824033@qq.com
///@date: 18-9-4
///修订历史：
////////////////////////////////////////////////////////////////////////////////

#ifndef RMDEMO_ROBOT_MODEL_H
#define RMDEMO_ROBOT_MODEL_H

#include "serial_port.h"
#include "serial_interface.h"
#include "usb_capture_with_thread.h"
#include "mind_vision.h"

typedef enum :unsigned char {
    ROBOT_MODE_AUTOAIM=0x00,
    ROBOT_MODE_MARKAIM=0x01,
    ROBOT_MODE_AUTO_PTZ=0x02,
    ROBOT_MODE_PLAYER_AIM = 0x03
} RobotMode;


class RobotModel {
public:
    RobotModel();
    ~RobotModel();
public:
    int init();

private://硬件资源
    UsbCaptureWithThread usb_capture_main_;
    UsbCaptureWithThread usb_capture_assist_;
    SerialInterface serial_interface_;
    int handle_camera_;
    MindVision mind_vision_;

private://机器人数据模型
    pthread_mutex_t data_mutex_ = PTHREAD_MUTEX_INITIALIZER; //互斥变量锁
    unsigned char robot_id_;//机器人id
    RobotMode current_mode_;//机器人当前运行模式
    float current_pitch_=0;//当前云台pitch角度
    float current_yaw_=0;//当前云台yaw角度
    bool enemy_is_red_;
public://硬件资源获取函数接口
    UsbCaptureWithThread* getpUsbCaptureMain();
    MindVision* getMvisionCapture();
    SerialInterface* getpSerialInterface();

public://机器人具体数据读写函数接口
    /*******系统框架调用接口，机器人数据更新*****/
    void mcuDataUpdate(float pitch,float yaw);
    /**************用户接口****************/
    unsigned char getRobotId();
    RobotMode getCurrentMode();
    void setCurrentMode(RobotMode robotMode);
    float getCurrentPitch();
    float getCurrentYaw();


};

#endif //RMDEMO_ROBOT_MODEL_H
