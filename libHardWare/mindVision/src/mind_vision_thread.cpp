#include "mind_vision_thread.h"

MindVisionThread::MindVisionThread(){
    isInit = false;
    isUpdate = false;
}

MindVisionThread::~MindVisionThread(){
    CameraUnInit(hCamera);
    free(g_pRgbBuffer);
}

int MindVisionThread::init(){
    CameraSdkInit(1);
    iStatus = CameraEnumerateDevice(&tCameraEnumList,&iCameraCounts);
    #ifdef DEBUG
	printf("state = %d\n", iStatus);
	printf("count = %d\n", iCameraCounts);
    #endif
    //没有连接设备
    if(iCameraCounts==0){
        return -1;
    }

    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList,-1,-1,&hCamera);

    //初始化失败
    #ifdef DEBUG
	printf("state = %d\n", iStatus);
    #endif
    if(iStatus!=CAMERA_STATUS_SUCCESS){
        return -1;
    }

    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(hCamera,&tCapability);
    g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);
    CameraPlay(hCamera);
    cv::FileStorage f;
    if(targetcolor == 0){ //红色
        f = FileStorage("../libHardWare/mindVision/res/light_red_config.yaml", cv::FileStorage::READ);
    } else {
        f = FileStorage("../libHardWare/mindVision/res/light_blue_config.yaml", cv::FileStorage::READ);
    }
    int width, height, frameSpeed, gamma, exposureTime, contrast, aeState;
    f["frame_speed"] >> frameSpeed;
    f["gamma"] >> gamma;
    f["exposure_time"] >> exposureTime;
    f["contrast"] >> contrast;
    f["ae_state"] >> aeState;
    f["resolution_width"] >> width;
    f["resolution_height"] >> height;
    f.release();
    setResolution(width, height);
       //帧率
    CameraSetFrameSpeed(hCamera, frameSpeed); //1
    //gamma值
    CameraSetGamma(hCamera, gamma); //20
    //曝光时间
    CameraSetExposureTime(hCamera, exposureTime); //1000
    //对比度
    CameraSetContrast(hCamera, contrast); //150
    //是否自动曝光
    CameraSetAeState(hCamera, aeState==0?FALSE:TRUE); //FALSE
    if(tCapability.sIspCapacity.bMonoSensor){
        channel=1;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
    }else{
        channel=3;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);
    }
    isInit = true;
    return 0;
}


int MindVisionThread::init(int width , int height , int color){
    cameraHeight = height;
    cameraWidth = width;
    targetcolor = color;
    int state = init();
    start();
    return state;
}
int MindVisionThread::setResolution(int width, int height){
    
    CameraGetImageResolution(hCamera, &tResolution);
    
    tResolution.iIndex = 0xFF;
    tResolution.iWidth = width;
    tResolution.iHeight = height;
    tResolution.iWidthFOV = width;
    tResolution.iHeightFOV = height;

    CameraSetImageResolution(hCamera, &tResolution);
    return 0;
}

int MindVisionThread::getImgRaw(){
    CameraReleaseImageBuffer(hCamera,pbyBuffer);
    if(CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS){
        CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);
        if (iplImage){
            cvReleaseImageHeader(&iplImage);
        }
        iplImage = cvCreateImageHeader(cvSize(sFrameInfo.iWidth,sFrameInfo.iHeight),IPL_DEPTH_8U,channel);
        cvSetData(iplImage,g_pRgbBuffer,sFrameInfo.iWidth*channel);
        pthread_mutex_lock(&imgMutex);
        img = cv::cvarrToMat(iplImage);
        pthread_mutex_unlock(&imgMutex);
        isUpdate = true;
        return 0;
    }
    return -1;
}

int MindVisionThread::getImg(Mat &src){
    if(!isUpdate){
        //等待100ms
        int timeCounter=0;
        while(!isUpdate&&timeCounter<100){
            usleep(1000);//1ms等待
            timeCounter++;
        }
        if(!isUpdate){
            return -3;//更新超时
        }
    }
    pthread_mutex_lock(&imgMutex);
    img.copyTo(src);//读mImg,加锁
    pthread_mutex_unlock(&imgMutex);
    if(!img.empty()&&(img.cols==cameraWidth)&&(img.rows==cameraHeight)){
        isUpdate= false;
        return 0;
    } else{
        return -1;
    }
}

void MindVisionThread::run(){
    while(true){
        if(isInit){
            getImgRaw();
        }
        else{
            usleep(30000);
        }
    }
}