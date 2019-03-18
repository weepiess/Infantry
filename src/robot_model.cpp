////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC ROBOMASTER2018      Model Code for robot
///ALL RIGHTS RESERVED
///@file:robot_model.cpp
///@brief: 机器人数据资源模型，包含机器人的所有硬件资源的统一管理，
/// 以及状态数据的统一管理，并提供机器人控制基本接口。
///@vesion 1.0
///@author: gezp
///@email: 1350824033@qq.com
///@date: 18-9-4
///修订历史：
////////////////////////////////////////////////////////////////////////////////

#include <unistd.h>
#include <robot_model.h>
#include <basic_tool.h>

using namespace cv;
using namespace std;

RobotModel::RobotModel() {

}
RobotModel::~RobotModel() {

}

int RobotModel::init(){
    string serial_path,video_path;
    //配置文件
    cv::FileStorage f("../res/main_config.yaml", cv::FileStorage::READ);
    f["robot_id"] >> robot_id_;//机器人id
    f["serial_path"] >> serial_path;//机器人串口路径
    f["capture_path"] >> video_path;//机器人摄像头路径
    f["enemy_is_red"] >> enemy_is_red_;//自瞄敌方颜色
    f.release();
    //初始化串口
    if(serial_interface_.init("/dev/ttyUSB0")==0) {
        cout<<"[robot model init ]: RobotSerialInterface init successed!"<<endl;
        //mSerialPort.ShowParam();
    } else{
        cout<<"[robot model init ]: RobotSerialInterface init failed!"<<endl;
    }

    handle_camera_ = mind_vision_.init();
    if(handle_camera_!=-1){
        mind_vision_.startPlay(handle_camera_, enemy_is_red_);
        cout << "[robot model init ]:UsbCapture init successed!" <<endl;
        usleep(1000000);//等待1s
    }else{
        cout << "[robot model init ]:UsbCapture init failed!" <<endl;
    }
    //  //初始化摄像头
    // if(usb_capture_main_.init("/dev/video0",1280,720)==0){
    //     cout << "[robot model init ]:UsbCapture Main init successed!" <<endl;
    //     usleep(1000000);//等待1s
    //     //mUsbCapture.infoPrint();
    // }else{
    //     cout << "[robot model init ]:UsbCapture Main init failed!" <<endl;
    // }
    // if(usb_capture_assist_.init("/dev/video1",1280,720)==0){
    //     cout << "[robot model init ]:UsbCaptureAssist init successed!" <<endl;
    //     usleep(1000000);//等待1s
    //     //mUsbCapture.infoPrint();
    // }else{
    //     cout << "[robot model init ]:UsbCapture Assist init failed!" <<endl;
    // }
    current_mode_=ROBOT_MODE_MARKAIM;

}


unsigned char RobotModel::getRobotId(){
    return robot_id_;
};


UsbCaptureWithThread* RobotModel::getpUsbCaptureMain() {
    return &usb_capture_main_;
}

MindVision* RobotModel::getMvisionCapture(){
    return &mind_vision_;
}

SerialInterface* RobotModel::getpSerialInterface() {
    return &serial_interface_;
}

//机器人数据接口
void RobotModel::setCurrentMode(RobotMode robotMode) {
    current_mode_=robotMode;
}


RobotMode RobotModel::getCurrentMode() {
    return current_mode_;
}


void RobotModel::mcuDataUpdate(float pitch, float yaw){
    pthread_mutex_lock(&data_mutex_);//加锁
    current_pitch_=pitch;
    current_yaw_=yaw;
    pthread_mutex_unlock(&data_mutex_);
}
float RobotModel::getCurrentPitch() {
    float tmp;
    pthread_mutex_lock(&data_mutex_);//加锁
    tmp=current_pitch_;
    pthread_mutex_unlock(&data_mutex_);
    return tmp;
}
float RobotModel::getCurrentYaw() {
    float tmp;
    pthread_mutex_lock(&data_mutex_);//加锁
    tmp=current_yaw_;
    pthread_mutex_unlock(&data_mutex_);
    return tmp;
}
