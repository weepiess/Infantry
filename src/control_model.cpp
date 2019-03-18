////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC ROBOMASTER2018      Model Code for robot
///ALL RIGHTS RESERVED
///@file:control_model.cpp
///@brief: robot 控制模型，包含对所有应用的管理，创建应用，并改变应用，以及监听操作手
/// 的指令，并处理。
///@vesion 1.0
///@author: gezp
///@email: 1350824033@qq.com
///@date: 18-9-4
///修订历史：增加云台自动化功能,修改自瞄逻辑. by PC 19-2-13
////////////////////////////////////////////////////////////////////////////////
#include "image_tool.h"
#include "control_model.h"
#include "basic_tool.h"
#include "serial_port_debug.h"
#include "usb_capture_with_thread.h"
#include "fstream"

using namespace cv;

ControlModel::ControlModel(){}

ControlModel::~ControlModel(){}

void ControlModel::init(RobotModel* robotModel){
    robot_model_=robotModel;
    
    //配置文件
    //初始模式初始化
    set_mode_=ROBOT_MODE_AUTOAIM;
    cap_ = robot_model_->getMvisionCapture();
    interface_ = robot_model_->getpSerialInterface();
    aim_assist_.init("../model4.pb");
    auto_aim_.init(&aim_assist_);

}
//串口数据接收处理入口
void ControlModel::serialListenDataProcess(SerialPacket recvPacket) {
//复杂自定义数据包，需要自定义析单独处理
    //debug
    //SerialPortDebug::testSerialPortListenPrint(recvPacket);
    unsigned char CMD= recvPacket.getCMD();
   // cout<<"CMD:"<<(int)CMD<<edl;
    if(CMD_SERIAL_ABS_YUNTAI_DELTA==CMD){
        //底层数据更新,pitch/yaw
        robot_model_->mcuDataUpdate(recvPacket.getFloatInBuffer(2),recvPacket.getFloatInBuffer(6));
    } else if(CMD_SERIAL_MINIPC_SHUTDOWN==CMD){
        //关机命令
        cout << "shutdown!!!!!!!!!!" << endl;
        system("shutdown -h now");
    }else if(CMD_SERIAL_MODE_CAHNGE==CMD){
        set_mode_ = RobotMode(recvPacket.getFloatInBuffer(2));
    }
    
}

void ControlModel::processFSM(){
    //模式切换预处理
    if(set_mode_!=robot_model_->getCurrentMode()){
        robot_model_->setCurrentMode(set_mode_);
        switch (set_mode_){
            case ROBOT_MODE_AUTOAIM:{
                auto_ptz_.join();
                recognize_wiggle_.join();
                auto_aim_.setEnemyColor(BaseAim::color_red);
                recognize_wiggle_.init();
                break;
            }
            case ROBOT_MODE_AUTO_PTZ:{
                recognize_wiggle_.join();
                interface_->YunTaiAbsSet(0,180);
                usleep(100);
                auto_ptz_.setCurrentAngel(robot_model_->getCurrentYaw()); 
                auto_ptz_.setScanDirection(AutoPTZ::CLOCKWISE);
                auto_ptz_.setScanSpeed(AutoPTZ::SLOW);
                auto_ptz_.init();
                break;
            }
            case ROBOT_MODE_PLAYER_AIM:{
                auto_ptz_.join();
                recognize_wiggle_.join();
                auto_aim_.setEnemyColor(BaseAim::color_blue);
                // mthread.join();
                // mthread.init(auto_aim_);
                // mthread.start();
                break;
            }
            case ROBOT_MODE_MARKAIM:{
                    board_serial_.breakUp();
                break;
            }
        }
    }
 
    //模式运行
    switch (robot_model_->getCurrentMode()){
        case ROBOT_MODE_AUTOAIM:{

            aim(false);
            break;
        }
        case ROBOT_MODE_AUTO_PTZ:{
            aim(true);
            break;
        }
        case ROBOT_MODE_PLAYER_AIM:{

            playerAim();
        }
        case ROBOT_MODE_MARKAIM:{
            mark();
            break;
        }
        default:
            cout<<"[aiProcess]mode error"<<endl;
            break;
    }

}

void ControlModel::aim(bool is_shoot_control){
        int start = basic_tool_.currentTimeMsGet();
        Mat src1;
        Mat src2;
        vector<RotatedRect> pre_armor_lamps;
        vector<RotatedRect> real_armor_lamps;
        Point2f current_angle;
        cap_ = robot_model_->getMvisionCapture();
        if(cap_->getImg(src1)!=0) cout<<"src is error"<<endl;
        // char c = waitKey(1);
        // cap_->adjustParams(c);
        interface_ = robot_model_->getpSerialInterface();
        interface_->getAbsYunTaiDelta();
        current_angle.x = robot_model_->getCurrentPitch();
        current_angle.y = robot_model_->getCurrentYaw();
        Point2f angle;
        if(auto_aim_.setImage(src1)){
            auto_aim_.findLampRect(pre_armor_lamps);
            auto_aim_.matchLamps(pre_armor_lamps,real_armor_lamps);
            auto_aim_.selectArmor(real_armor_lamps);
            int finish = basic_tool_.currentTimeMsGet();
            bool if_shoot=false;
            if(auto_aim_.aim(src1, current_angle.x, current_angle.y, angle,0,if_shoot,finish-start )==BaseAim::AIM_TARGET_FOUND){
                int end = basic_tool_.currentTimeMsGet();
                cout<<"time cost: "<<end-start<<endl;
                interface_->YunTaiDeltaSet(angle.x, angle.y);
                cout<<"angle1 "<<angle<<endl;
                unsigned char num=0x01;
                if(if_shoot)
	                interface_->YunTaiShoot(num);
        }
    //    imshow("src",src1);
    //    waitKey(1);
    }        
}

void ControlModel::autoPTZControl(){
    int start = basic_tool_.currentTimeMsGet();
    Mat src1;
    vector<RotatedRect> pre_armor_lamps;
    vector<RotatedRect> real_armor_lamps;
    Point2f current_angle;
    cap_ = robot_model_->getMvisionCapture();
    if(cap_->getImg(src1)!=0) cout<<"src is error"<<endl;
    interface_->getAbsYunTaiDelta();
    current_angle.x = robot_model_->getCurrentPitch();
    current_angle.y = robot_model_->getCurrentYaw();
    Point2f angle;
    if(auto_aim_.setImage(src1)){
        auto_aim_.findLampRect(pre_armor_lamps);
        auto_aim_.matchLamps(pre_armor_lamps,real_armor_lamps);
            
        auto_aim_.selectArmor(real_armor_lamps);
        int finish = basic_tool_.currentTimeMsGet();
        bool if_shoot=false;
        if(auto_aim_.aim(src1, current_angle.x, current_angle.y, angle,1,if_shoot,finish-start )==BaseAim::AIM_TARGET_FOUND){
            auto_ptz_.isTargetFind(true);
            interface_->YunTaiDeltaSet(angle.x, angle.y);
            cout<<"angle1 "<<angle<<endl;
            unsigned char num=0x01;
            if(if_shoot)    interface_->YunTaiShoot(num);
        }else{
            auto_ptz_.isTargetFind(false);
            auto_ptz_.threadResume();
            auto_ptz_.setCurrentAngel(current_angle.y);
            interface_->YunTaiAbsSet(0,current_angle.y);
            interface_->YunTaiDeltaSet(0,auto_ptz_.getAngle());
        }  
    }          
}

void ControlModel::playerAim(){
    int start = basic_tool_.currentTimeMsGet();
    Mat src1;
    vector<RotatedRect> pre_armor_lamps;
    vector<RotatedRect> real_armor_lamps;
    Point2f current_angle;
    cap_ = robot_model_->getMvisionCapture();
    if(cap_->getImg(src1)!=0) cout<<"src is error"<<endl;
    // char c = waitKey(1);
    // cap_->adjustParams(c);
    interface_ = robot_model_->getpSerialInterface();
    interface_->getAbsYunTaiDelta();
    current_angle.x = robot_model_->getCurrentPitch();
    current_angle.y = robot_model_->getCurrentYaw();
    Point2f angle;
    if(auto_aim_.setImage(src1)){
        auto_aim_.findLampRect(pre_armor_lamps);
        auto_aim_.matchLamps(pre_armor_lamps,real_armor_lamps);    
        auto_aim_.selectArmorH(real_armor_lamps);
        int finish = basic_tool_.currentTimeMsGet();
        bool if_shoot=false;
        if(auto_aim_.aim(src1, current_angle.x, current_angle.y, angle,1,if_shoot,finish-start )==BaseAim::AIM_TARGET_FOUND){
            int end = basic_tool_.currentTimeMsGet();
            cout<<"time cost: "<<end-start<<endl;
            interface_->YunTaiDeltaSet(angle.x, angle.y);
            cout<<"angle1 "<<angle<<endl;
        }
    }
}

void ControlModel::mark(){
    Point3f  point = board_serial_.returnVal();
    Point2f current_angle;
    current_angle.x = robot_model_->getCurrentPitch();
    current_angle.y = robot_model_->getCurrentYaw();
    // auto_aim_.calPitchAndYaw(point.x,point.y,point.z);
    
}

