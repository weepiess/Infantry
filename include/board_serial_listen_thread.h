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
    void Sleep();
    void BreakUp();
    Point3f ReturnVal();
    void sendStartMessage();

private:
    SerialInterface mSerial;

private:
    pthread_cond_t  cond = PTHREAD_COND_INITIALIZER;
    pthread_mutex_t signalMutex = PTHREAD_MUTEX_INITIALIZER;
    bool wake_up;
    Point3f Repoint;
    
    
};
#endif