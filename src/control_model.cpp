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
    pRobotModel=robotModel;
    
    //配置文件
    //初始模式初始化
    mSetMode=ROBOT_MODE_AUTOAIM;
    cap = pRobotModel->getMvisionCapture();
    interface = pRobotModel->getpSerialInterface();
    aim_assistant.init("../model4.pb");
    autoAim.init(&aim_assistant);

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
        pRobotModel->mcuDataUpdate(recvPacket.getFloatInBuffer(2),recvPacket.getFloatInBuffer(6));
    } else if(CMD_SERIAL_MINIPC_SHUTDOWN==CMD){
        //关机命令
        cout << "shutdown!!!!!!!!!!" << endl;
        system("shutdown -h now");
    }else if(CMD_SERIAL_MODE_CAHNGE==CMD){
        mSetMode = RobotMode(recvPacket.getFloatInBuffer(2));
    }
    
}

void ControlModel::processFSM(){
    //模式切换预处理
    if(mSetMode!=pRobotModel->getCurrentMode()){
        pRobotModel->setCurrentMode(mSetMode);
        switch (mSetMode){
            case ROBOT_MODE_AUTOAIM:{
                autoptz.join();
                autoAim.set_parameters(3,45,30,20);
                autoAim.setEnemyColor(BaseAim::color_red);
                break;
            }
            case ROBOT_MODE_AUTO_PTZ:{
                autotrn.join();
                interface->YunTaiAbsSet(0,180);
                usleep(100);
                autoptz.setCurrentAngel(pRobotModel->getCurrentYaw()); 
                autoptz.setScanDirection(AutoPTZ::CLOCKWISE);
                autoptz.setScanSpeed(AutoPTZ::SLOW);
                autoptz.init();
                break;
            }
            case ROBOT_MODE_PLAYER_AIM:{
                autoptz.join();
                autotrn.join();
                autoAim.set_parameters(3,45,30,20);
                autoAim.setEnemyColor(BaseAim::color_red);
                // mthread.join();
                // mthread.init(autoAim);
                // mthread.start();
                break;
            }
            case ROBOT_MODE_MARKAIM:{
                    mBoardSerial.BreakUp();
                break;
            }
        }
    }
 
    //模式运行
    switch (pRobotModel->getCurrentMode()){
        case ROBOT_MODE_AUTOAIM:{

            Aim(false);
            break;
        }
        case ROBOT_MODE_AUTO_PTZ:{
            Aim(true);
            break;
        }
        case ROBOT_MODE_PLAYER_AIM:{

            playerAim();
        }
        case ROBOT_MODE_MARKAIM:{
            Mark();
            break;
        }
        default:
            cout<<"[aiProcess]mode error"<<endl;
            break;
    }

}

void ControlModel::Aim(bool is_shoot_control){
        int start = basic_tool.currentTimeMsGet();
        Mat src1;
        Mat src2;
        vector<RotatedRect> pre_armor_lamps;
        vector<RotatedRect> real_armor_lamps;
        Point2f current_angle;
        MindVision* cap = pRobotModel->getMvisionCapture();
        if(cap->getImg(src1)!=0) cout<<"src is error"<<endl;
        char c = waitKey(1);
        cap->adjustParams(c);
        SerialInterface *interface = pRobotModel->getpSerialInterface();
        interface->getAbsYunTaiDelta();
        current_angle.x = pRobotModel->getCurrentPitch();
        current_angle.y = pRobotModel->getCurrentYaw();
        Point2f angle;
        if(autoAim.setImage(src1)){
            autoAim.findLamp_rect(pre_armor_lamps);
            autoAim.match_lamps(pre_armor_lamps,real_armor_lamps);
            
            autoAim.select_armor(real_armor_lamps);
            int finish = basic_tool.currentTimeMsGet();
            bool if_shoot=false;
            if(autoAim.aim(src1, current_angle.x, current_angle.y, angle,1,if_shoot,finish-start )==BaseAim::AIM_TARGET_FOUND){
                int end = basic_tool.currentTimeMsGet();
                cout<<"time cost: "<<end-start<<endl;
                interface->YunTaiDeltaSet(angle.x, angle.y);
                cout<<"angle1 "<<angle<<endl;
                unsigned char num=0x01;
                if(if_shoot)
	                interface->YunTaiShoot(num);
        }
        imshow("src",src1);
        waitKey(1);

    }        
}

void ControlModel::AutoPTZControl(){
    int start = basic_tool.currentTimeMsGet();
    Mat src1;
    vector<RotatedRect> pre_armor_lamps;
    vector<RotatedRect> real_armor_lamps;
    Point2f current_angle;
    MindVision* cap = pRobotModel->getMvisionCapture();
    if(cap->getImg(src1)!=0) cout<<"src is error"<<endl;
    interface->getAbsYunTaiDelta();
    current_angle.x = pRobotModel->getCurrentPitch();
    current_angle.y = pRobotModel->getCurrentYaw();
    Point2f angle;
    if(autoAim.setImage(src1)){
        autoAim.findLamp_rect(pre_armor_lamps);
        autoAim.match_lamps(pre_armor_lamps,real_armor_lamps);
            
        autoAim.select_armor(real_armor_lamps);
        int finish = basic_tool.currentTimeMsGet();
        bool if_shoot=false;
        if(autoAim.aim(src1, current_angle.x, current_angle.y, angle,1,if_shoot,finish-start )==BaseAim::AIM_TARGET_FOUND){
            autoptz.isTargetFind(true);
            interface->YunTaiDeltaSet(angle.x, angle.y);
            cout<<"angle1 "<<angle<<endl;
            unsigned char num=0x01;
            if(if_shoot)    interface->YunTaiShoot(num);
        }else{
            autoptz.isTargetFind(false);
            autoptz.threadResume();
            autoptz.setCurrentAngel(current_angle.y);
            interface->YunTaiAbsSet(0,current_angle.y);
            interface->YunTaiDeltaSet(0,autoptz.getAngle());
        }  
    }          
}

void ControlModel::playerAim(){
    int start = basic_tool.currentTimeMsGet();
    Mat src1;
    vector<RotatedRect> pre_armor_lamps;
    vector<RotatedRect> real_armor_lamps;
    Point2f current_angle;
    MindVision* cap = pRobotModel->getMvisionCapture();
    if(cap->getImg(src1)!=0) cout<<"src is error"<<endl;
    char c = waitKey(1);
    cap->adjustParams(c);
    SerialInterface *interface = pRobotModel->getpSerialInterface();
    interface->getAbsYunTaiDelta();
    current_angle.x = pRobotModel->getCurrentPitch();
    current_angle.y = pRobotModel->getCurrentYaw();
    Point2f angle;
    if(autoAim.setImage(src1)){
        autoAim.findLamp_rect(pre_armor_lamps);
        autoAim.match_lamps(pre_armor_lamps,real_armor_lamps);    
        autoAim.selectArmorH(real_armor_lamps);
        int finish = basic_tool.currentTimeMsGet();
        bool if_shoot=false;
        if(autoAim.aim(src1, current_angle.x, current_angle.y, angle,1,if_shoot,finish-start )==BaseAim::AIM_TARGET_FOUND){
            int end = basic_tool.currentTimeMsGet();
            cout<<"time cost: "<<end-start<<endl;
            interface->YunTaiDeltaSet(angle.x, angle.y);
            cout<<"angle1 "<<angle<<endl;
        }
    }
}

void ControlModel::Mark(){
    Point3f  Ppoint = mBoardSerial.ReturnVal();
}

