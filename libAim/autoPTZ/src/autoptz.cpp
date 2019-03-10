#include "autoptz.h"

AutoPTZ::AutoPTZ(){}
AutoPTZ::~AutoPTZ(){}

void AutoPTZ::init(){
    current_yaw  = 180;
    is_target_found = false;
    speed_level = SLOW;
    scan_direction = CLOCKWISE;
    start();
}

float AutoPTZ::getAngle(){
    float yaw_angle;
    pthread_mutex_lock(&controlMutex);
    yaw_angle = yaw_tmp;
    pthread_mutex_unlock(&controlMutex);
    return yaw_angle;
}

void AutoPTZ::setCurrentAngel(float yaw){
    pthread_mutex_lock(&controlMutex);
    current_yaw = yaw;
    pthread_mutex_unlock(&controlMutex);
}

void AutoPTZ::setScanSpeed(ScanSpeed speed){
    speed_level = speed;
}

void AutoPTZ::setScanDirection(ScanDirection direction){
    scan_direction = direction;
}

void AutoPTZ::isTargetFind(bool flag){
    is_target_found = flag;
}

void AutoPTZ::AutoFunc(){
    while(1){
        pthread_mutex_lock(&controlMutex);
        while(is_target_found){
            pthread_cond_wait(&cond, &controlMutex);
        }
        pthread_mutex_unlock(&controlMutex);
        switch(speed_level){
            case FAST:{
                if(scan_direction==CLOCKWISE){
                    pthread_mutex_lock(&controlMutex);
                    yaw_tmp = 2;
                    current_yaw +=2;
                    pthread_mutex_unlock(&controlMutex);
                }else{
                    pthread_mutex_lock(&controlMutex);
                    yaw_tmp = -2;
                    current_yaw -=2;
                    pthread_mutex_unlock(&controlMutex);
                }   
            }
            case MEDIUM:{
                if(scan_direction==CLOCKWISE){
                    pthread_mutex_lock(&controlMutex);
                    yaw_tmp = 1;
                    current_yaw +=1;
                    pthread_mutex_unlock(&controlMutex);
                }else{
                    pthread_mutex_lock(&controlMutex);
                    yaw_tmp = -1;
                    current_yaw -=1;
                    pthread_mutex_unlock(&controlMutex);
                }   
            }
            case SLOW:{
                if(scan_direction==CLOCKWISE){
                    pthread_mutex_lock(&controlMutex);
                    yaw_tmp = 0.5;
                    current_yaw +=0.5;
                    pthread_mutex_unlock(&controlMutex);
                }else{
                    pthread_mutex_lock(&controlMutex);
                    yaw_tmp = -0.5;
                    current_yaw -=0.5;
                    pthread_mutex_unlock(&controlMutex);
                }   
            }
        }
    }
}

void AutoPTZ::threadResume(){
    pthread_mutex_lock(&controlMutex);
    //pthread_cond_broadcast(&cond);
    pthread_cond_signal(&cond);
    pthread_mutex_unlock(&controlMutex);
}

void AutoPTZ::run(){
    while(true){
        AutoFunc();
    }
}