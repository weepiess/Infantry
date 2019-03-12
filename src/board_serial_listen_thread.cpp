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
    while(wake_up&&mSerial.isOpen()){
        while(mSerial.dataRecv32(mSerialPacket)==0){
            u_char CMD = mSerialPacket.getCMD();
            if(CMD ==CMD_SERIAL_BOARD_REV){
                    Repoint = Point3f(mSerialPacket.getIntInBuffer(2),mSerialPacket.getIntInBuffer(6),mSerialPacket.getIntInBuffer(10));
            }
        }
    }
    
}

Point3f BoardSerialListenThread::ReturnVal(){
    return Repoint;
}