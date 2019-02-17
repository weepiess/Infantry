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
///修订历史：
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
    autoAim = new AutoAim(1280, 720);
    //配置文件
    //初始模式初始化
    mSetMode=ROBOT_MODE_AUTOAIM;
    //aim_assist.init("../res/model3.pb");
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
    } 
    
}

void ControlModel::processFSM(){
    //模式切换预处理
    if(mSetMode!=pRobotModel->getCurrentMode()){
        pRobotModel->setCurrentMode(mSetMode);
        switch (mSetMode){
            case ROBOT_MODE_AUTOAIM:{
                autoAim->set_parameters(3,45,30,20);
                autoAim->setEnemyColor(BaseAim::color_red);
                mthread.join();
                mthread.init(autoAim);
                mthread.start();
                break;
            }
        }
    }
 
    //模式运行
    switch (pRobotModel->getCurrentMode()){
        case ROBOT_MODE_AUTOAIM:{
            Aim();
            break;
        }
        default:
            cout<<"[aiProcess]mode error"<<endl;
            break;
    }

}

void ControlModel::Aim(){
        int start = basic_tool.currentTimeMsGet();
        Mat src1;
        Mat src2;
        vector<RotatedRect> pre_armor_lamps;
        vector<RotatedRect> real_armor_lamps;
        Point2f current_angle;
        MindVision* cap = pRobotModel->getMvisionCapture();
        //UsbCaptureWithThread* cap2 = pRobotModel->getpUsbCaptureAssist();
        //if(cap2->getImg(src2)!=0) cout<<"src is error"<<endl;
        if(cap->getImg(src1)!=0) cout<<"src is error"<<endl;
        char c = waitKey(1);
        cap->adjustParams(c);
        //src2 = imread("../res/2.png");
        SerialInterface *interface = pRobotModel->getpSerialInterface();
        interface->getAbsYunTaiDelta();
        current_angle.x = pRobotModel->getCurrentPitch();
        current_angle.y = pRobotModel->getCurrentYaw();
        Point2f angle;
        
        //int result=aim_assist.check_armor(src2);
        //cout<<finish-start<<" time"<<endl;

        if(autoAim->setImage(src1)){
            autoAim->findLamp_rect(pre_armor_lamps);
            autoAim->match_lamps(pre_armor_lamps,real_armor_lamps);
            
            autoAim->select_armor(real_armor_lamps);
            int finish = basic_tool.currentTimeMsGet();
            bool if_shoot=false;
            if(autoAim->aim(src1, current_angle.x, current_angle.y, angle,1,if_shoot,finish-start )==BaseAim::AIM_TARGET_FOUND){
                 //rectangle(src2, autoAim->armor, Scalar(255,0,0), 2);
                 //int finish = basic_tool.currentTimeMsGet();
                 //cout<<finish-start<<"   time"<<endl;
                interface->YunTaiDeltaSet(angle.x, angle.y);
                cout<<"angle1 "<<angle<<endl;
                ofstream outfile1("/home/weepies/output.txt", ios::ate);
                outfile1 << angle.y << endl;
                unsigned char num=0x01;
                if(if_shoot)
	                interface->YunTaiShoot(num);
                //int result=aim_assist.check_armor(src2(autoAim->armor));
                // if(result!=-1){
                //     putText(src2,to_string(result),Point(autoAim->armor.x,autoAim->armor.y-30),cv::FONT_HERSHEY_COMPLEX,2,2);
                // }
                // Mat s ;
                // cvtColor(src2(autoAim->armor),s,COLOR_BGR2GRAY);
                //  cv::resize(s,s,cv::Size(32,32));

            }
            //imshow("armor",src2(autoAim->armor));
            //imshow("src2",src2);
            //waitKey(1);
        }    
}

//异步装甲板确认 用于如果神经网络运行时间过长
//imwrite("aror/"+to_string(autoAim->armor.x)+to_string(autoAim->armor.y)+".png",s);
 //     if(need_check&&!aim_assist.is_checked){
//           tmp = std::async(&Aim_assistant::check_armor,aim_assist,src2);//异步执行装甲板确认函数
//           need_check=true;
//       }
//       if(aim_assist.is_checked){ 
//          armor_id = tmp.get();
//          aim_assist.is_checked=false; 
//       }
//  //    interface->YunTaiDeltaSet(angle.x, angle.y);
//   }else {
//       need_check=true;
//       aim_assist.is_checked=false;
//    }
