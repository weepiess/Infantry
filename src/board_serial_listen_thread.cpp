#include "board_serial_listen_thread.h"

BoardSerialListenThread::BoardSerialListenThread(){
    wake_up = false;
}
BoardSerialListenThread::~BoardSerialListenThread(){}

void BoardSerialListenThread::Sleep(){
    wake_up = false;
}

void BoardSerialListenThread::BreakUp(){

    wake_up = true;
    pthread_mutex_lock(&signalMutex);
    //pthread_cond_broadcast(&cond);
    pthread_cond_signal(&cond);
    pthread_mutex_unlock(&signalMutex);
}

bool BoardSerialListenThread::init(string path){
    if(mSerial.init(path) == 0){
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
        if(mSerial.isOpen()){
        pthread_mutex_lock(&signalMutex);   
        while(wake_up){
            pthread_cond_wait(&cond, &signalMutex);
        }
        pthread_mutex_unlock(&signalMutex);
            while(mSerial.dataRecv32(mSerialPacket)==0){
                u_char CMD = mSerialPacket.getCMD();
                if(CMD ==CMD_SERIAL_BOARD_REV){
                    pthread_mutex_lock(&signalMutex);
                    Repoint = Point3f(mSerialPacket.getIntInBuffer32(2),mSerialPacket.getIntInBuffer32(6),mSerialPacket.getIntInBuffer32(10));
                    pthread_mutex_unlock(&signalMutex);
                }
            }
        }
    }
}

void BoardSerialListenThread::sendStartMessage(){
    mSerial.BoardCommand();
}

Point3f BoardSerialListenThread::ReturnVal(){
    Point3f points;
    pthread_mutex_lock(&signalMutex);
    points = Repoint;
    pthread_mutex_unlock(&signalMutex);
    return points;
}