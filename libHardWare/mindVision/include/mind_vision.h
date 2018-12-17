#ifndef MIND_VISION_H
#define MIND_VISION_H

#include "CameraApi.h" //相机SDK的API头文件

#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include "function_thread.h"

//#define CAMERA_PARAMS_ADJUST

using namespace cv;
using namespace std;

class MindVision{
public:
    MindVision();
    ~MindVision();

public:
    /** 相机初始化，直接获得句柄并让sdk开始工作（该情况只适用于只有一个相机的情况）
     * @return: int, 错误码，-1为错误，0为正确
     */
    int init(bool is_enemy_red_);

    /** 设置相机的分辨率
     * @param: width, 横向像素
     * @param: height, 纵向像素
     * @return: int, 错误码
     */
    int setResolution(int width, int height);

    /** 是否暂停采集
     * @param: is_capture, 是否采集
     */
    void setIsCapture(bool is_capture);

    /** 得到图像
     * @param: src, 得到的图像
     * @return: int, 错误码
     */
    int getImg(Mat &src);

private:
    /** 在另外一个线程中运行，不断更新内部Mat
     * @param:
     * @return:
     */
    void updateInThread();

    /** 采集原始图片并更新Mat
     * @param:
     */
    int getRawImage();

    unsigned char           * g_pRgbBuffer;       //处理后数据缓存区
    int                     iCameraCounts = 1;    //找到的相机的个数
    int                     iStatus=0;            //状态返回值
    int                     hCamera=-1;           //相机句柄
    tSdkCameraDevInfo       tCameraEnumList;      //设备列表信息
    tSdkCameraCapbility     tCapability;          //设备描述信息
    tSdkFrameHead           sFrameInfo;           //每一帧的头指针
    BYTE*			        pbyBuffer;            //数据缓冲区
    IplImage                *iplImage = NULL;     //图像的矩阵存储形式     
    int                     channel=3;            //通道数
    tSdkImageResolution     tResolution;          //像素

    Mat bufferImage; //缓冲图像，用于线程更新使用

    std::mutex imgMutex;

    std::atomic<bool> isUpdate; //更新标志位

    bool isCapture; //是否采集图像

    bool is_enemy_red; //敌方颜色，用于重连时初始化

    FunctionThread function_thread; //线程函数

    #ifdef CAMERA_PARAMS_ADJUST
private:
    /** 根据传入的字符调整不同的参数
     * @param: c, 键盘输入的字符
     * @return: void
     */
    void adjustParams(unsigned char c);

    /** 当前的操作是作用在哪个相机元素上
     * "f"：frameSpeed  "g"：gamma  "e"：exposureTime  "c"：contrast  "a":aeState
     */
    unsigned char currOperation;

    /** 用于调整相机参数的临时变量*/
    int index, gamma, exposureTime, contrast, aeState, resolution_width, resolution_height;
    #endif
};

#endif