#ifndef BOARD_SERIAL_LISTEN_THREAD
#define BOARD_SERIAL_LISTEN_THREAD

#include "base_thread.h"
#include "serial_interface.h"
#include "serial_packet.h"

class BoardSerialListenThread:public BaseThread{

public:
    BoardSerialListenThread();
    ~BoardSerialListenThread();

public:
    bool init(string path);
    void run();
    void Sleep();
    void BreakUp();
    float *ReturnVal();

private:
    bool wake_up;
    SerialInterface mSerial;
    float Spoint[8];
};
#endif