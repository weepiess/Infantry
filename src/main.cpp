////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC ROBOMASTER2018      MainFun Code for robot
///ALL RIGHTS RESERVED
///@file:main.cpp
///@brief: 无。
///@vesion 1.0
///@author: gezp
///@email: 1350824033@qq.com
///@date: 18-9-4
///修订历史：
////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include "control_model.h"
#include "serial_listen_thread.h"
#include "serial_port_debug.h"
#include "log.h"

using  namespace std;

int main(int argc, char *argv[]){
    GLogWrapper glog_wrapper(argv[0]);
    RobotModel robotModel;
    cout<<"[robot init]robot model start to initialize!"<<endl;
    robotModel.init();
    cout<<"[robot init]robot control model start to initializ!"<<endl;
    usleep(10000);
    ControlModel controlModel;
    controlModel.init(&robotModel);
    cout<<"[robot init]robot serial port start to listen!"<<endl;
    SerialListenThread serialListenThread;
    serialListenThread.init(&robotModel,&controlModel);
    serialListenThread.start();
    cout<<"[robot init]robot init end!"<<endl;
    //debug模块
    //SerialPortDebug serialPortDebug;
    //serialPortDebug.init(robotModel.getpSerialInterface());
    //serialPortDebug.testSerialPort();
    //主逻辑
    while(true){
        controlModel.processFSM();
    }
    serialListenThread.join();
    cout<<"error end!"<<endl;
    getchar();//防止监听线程意外结束直接退出。
    return 0;
}


/*
VideoCapture capture;
    Mat frame;
    Mat mask;
    Mat Mask;
    Mat temp;
    Mat channel[3];
    Mat result;
    Mat result2;
    Mat diff;
    frame= capture.open("/home/weepies/RM/INF/res/output(1).avi");
    while(capture.read(frame)){
        Mat watch=Mat::zeros(frame.size(),CV_8UC3);
        cvtColor(frame,mask,COLOR_BGR2GRAY);
        split(frame,channel);
       Mask=channel[2]-channel[1];
       diff=channel[2];
       GaussianBlur(temp, temp, Size(5,5), 3);
       threshold(diff,diff,100,255,THRESH_BINARY);
       threshold(Mask,temp,40,255,THRESH_BINARY);
       Mat element = getStructuringElement( MORPH_ELLIPSE, Size(10, 10));
        dilate( diff, diff, element,Point(-1,-1),5);   
       bitwise_and(temp, diff, result);
       morphologyEx(result,result,MORPH_CLOSE,element);
    //    threshold(result,result2,0,255,THRESH_OTSU);
       //GaussianBlur(result2,result2,Size(5,5),5);
    //    medianBlur(result2,result2,3);
        // Mat element = getStructuringElement( MORPH_ELLIPSE, Size(5, 3));
        // dilate( result2, result2, element,Point(-1,-1),5);   
        // erode(result2,result2,element,Point(-1,-1),5);
        //Canny(result2,result2,0,255);
        //threshold(mask,mask,0,255,THRESH_BINARY_INV);
        //controlModel.processFSM();
        //imshow("channel[2]",channel[2]-channel[0]-channel[1]);
        imshow("result",result);
        // imshow("mask",mask);
        // imshow("output",frame);
        waitKey(1);
        vector<vector<Point> > contours;
        vector<Vec4i> hierarcy;
        vector<RotatedRect> Rrect;
        vector<Rect> Rrect2;
        findContours(result, contours, hierarcy, CV_RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
        RotatedRect rtemp;
        Rect rtemp2;
        if(contours.size()>0){
            cout<<contours.size()<<"   size   "<<endl;
            for(int i=0; i<contours.size(); i++){
                rtemp=minAreaRect(contours[i]);
                rtemp2=boundingRect(contours[i]);
                Rrect.push_back(minAreaRect(contours[i]));
                Rrect2.push_back(boundingRect(contours[i]));
                Point2f vertices[4];      //定义4个点的数组
                rtemp.points(vertices);   //将四个点存储到vertices数组中
                // for (int i = 0; i < 4; i++)
                //     line(watch, vertices[i], vertices[(i+1)%4], Scalar(0,255,0));
                rectangle(watch,rtemp2,Scalar(0,0,255),3);
                    
            }
            for(int i=0; i<Rrect2.size()-1; i++){
                //cout<<"distance  "<<sqrt(pow(Rrect[i].center.x-Rrect[i+1].center.x,2)+pow(Rrect[i].center.y-Rrect[i+1].center.y,2));
                float distance=sqrt(pow((Rrect2[i].tl().x+Rrect2[i].br().x)/2-(Rrect2[i+1].tl().x+Rrect2[i+1].br().x)/2,2)+pow((Rrect2[i].tl().y+Rrect2[i].br().y)/2-(Rrect2[i+1].tl().y+Rrect2[i+1].br().y)/2,2));
                if(distance<=80){
                    circle(watch,Point(((Rrect2[i].tl().x+Rrect2[i].br().x)/2+(Rrect2[i+1].tl().x+Rrect2[i+1].br().x)/2)/2,((Rrect2[i].tl().y+Rrect2[i].br().y)/2+(Rrect2[i+1].tl().y+Rrect2[i+1].br().y)/2)/2),4,Scalar(255,0,0),4);
                }
            }
            imshow("watch",watch);
                waitKey(1);
        }
       
    }

    capture.release();
    //serialListenThread.join();
    //cout<<"error end!"<<endl;
    //getchar();//防止监听线程意外结束直接退出。
    return 0;
}
*/
