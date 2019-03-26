#ifndef BOARD_SERIAL_LISTEN_THREAD
#define BOARD_SERIAL_LISTEN_THREAD

#include "base_thread.h"
#include "serial_interface.h"
#include "serial_packet.h"
#include "opencv2/opencv.hpp"
using namespace cv;
class BoardSerialListenThread:public BaseThread{

public:
    BoardSerialListenThread();
    ~BoardSerialListenThread();

public:
    bool init(string path);
    void run();
    void sleep();
    void breakUp();
    Point3f returnVal();
    void sendStartMessage();

private:
    SerialInterface serial_;

private:
    pthread_cond_t  cond_ = PTHREAD_COND_INITIALIZER;
    pthread_mutex_t signal_mutex_ = PTHREAD_MUTEX_INITIALIZER;
    bool wake_up_;
    Point3f get_point_;
    
    
};
#endif