#include "board_serial_listen_thread.h"

BoardSerialListenThread::BoardSerialListenThread(){
    wake_up_ = false;
}
BoardSerialListenThread::~BoardSerialListenThread(){}

void BoardSerialListenThread::sleep(){
    wake_up_ = false;
}

void BoardSerialListenThread::breakUp(){

    wake_up_ = true;
    pthread_mutex_lock(&signal_mutex_);
    //pthread_cond_broadcast(&cond_);
    pthread_cond_signal(&cond_);
    pthread_mutex_unlock(&signal_mutex_);
}

bool BoardSerialListenThread::init(string path){
    if(serial_.init(path) == 0){
        start();
        return true;
    }
    else{
        return false;
        join();
    }
}

void BoardSerialListenThread::run(){
    SerialPacket mSerialPacket;
    while(1){
        if(serial_.isOpen()){
        pthread_mutex_lock(&signal_mutex_);   
        while(wake_up_){
            pthread_cond_wait(&cond_, &signal_mutex_);
        }
        pthread_mutex_unlock(&signal_mutex_);
            while(serial_.dataRecv32(mSerialPacket)==0){
                u_char CMD = mSerialPacket.getCMD();
                if(CMD ==CMD_SERIAL_BOARD_REV){
                    pthread_mutex_lock(&signal_mutex_);
                    get_point_ = Point3f(mSerialPacket.getIntInBuffer32(2),mSerialPacket.getIntInBuffer32(6),mSerialPacket.getIntInBuffer32(10));
                    pthread_mutex_unlock(&signal_mutex_);
                }
            }
        }
    }
}

void BoardSerialListenThread::sendStartMessage(){
    serial_.BoardCommand();
}

Point3f BoardSerialListenThread::returnVal(){
    Point3f points;
    pthread_mutex_lock(&signal_mutex_);
    points = get_point_;
    pthread_mutex_unlock(&signal_mutex_);
    return points;
}