#include "autoptz.h"

AutoPTZ::AutoPTZ(){}
AutoPTZ::~AutoPTZ(){}

void AutoPTZ::init(){
    current_yaw_  = 180;
    is_target_found_ = false;
    speed_level_ = SLOW;
    scan_direction_ = CLOCKWISE;
    start();
}

float AutoPTZ::getAngle(){
    float yaw_angle;
    pthread_mutex_lock(&control_mutex_);
    yaw_angle = yaw_tmp_;
    pthread_mutex_unlock(&control_mutex_);
    return yaw_angle;
}

void AutoPTZ::setCurrentAngel(float yaw){
    pthread_mutex_lock(&control_mutex_);
    current_yaw_ = yaw;
    pthread_mutex_unlock(&control_mutex_);
}

void AutoPTZ::setScanSpeed(ScanSpeed speed){
    speed_level_ = speed;
}

void AutoPTZ::setScanDirection(ScanDirection direction){
    scan_direction_ = direction;
}

void AutoPTZ::isTargetFind(bool flag){
    is_target_found_ = flag;
}

void AutoPTZ::autoFunc(){
    while(1){
        pthread_mutex_lock(&control_mutex_);
        while(is_target_found_){
            pthread_cond_wait(&cond_, &control_mutex_);
        }
        pthread_mutex_unlock(&control_mutex_);
        switch(speed_level_){
            case FAST:{
                if(scan_direction_==CLOCKWISE){
                    pthread_mutex_lock(&control_mutex_);
                    yaw_tmp_ = 2;
                    current_yaw_ +=2;
                    pthread_mutex_unlock(&control_mutex_);
                }else{
                    pthread_mutex_lock(&control_mutex_);
                    yaw_tmp_ = -2;
                    current_yaw_ -=2;
                    pthread_mutex_unlock(&control_mutex_);
                }   
            }
            case MEDIUM:{
                if(scan_direction_==CLOCKWISE){
                    pthread_mutex_lock(&control_mutex_);
                    yaw_tmp_ = 1;
                    current_yaw_ +=1;
                    pthread_mutex_unlock(&control_mutex_);
                }else{
                    pthread_mutex_lock(&control_mutex_);
                    yaw_tmp_ = -1;
                    current_yaw_ -=1;
                    pthread_mutex_unlock(&control_mutex_);
                }   
            }
            case SLOW:{
                if(scan_direction_==CLOCKWISE){
                    pthread_mutex_lock(&control_mutex_);
                    yaw_tmp_ = 0.5;
                    current_yaw_ +=0.5;
                    pthread_mutex_unlock(&control_mutex_);
                }else{
                    pthread_mutex_lock(&control_mutex_);
                    yaw_tmp_ = -0.5;
                    current_yaw_ -=0.5;
                    pthread_mutex_unlock(&control_mutex_);
                }   
            }
        }
    }
}

void AutoPTZ::threadResume(){
    pthread_mutex_lock(&control_mutex_);
    //pthread_cond_broadcast(&cond_);
    pthread_cond_signal(&cond_);
    pthread_mutex_unlock(&control_mutex_);
}

void AutoPTZ::run(){
    while(true){
        autoFunc();
    }
}