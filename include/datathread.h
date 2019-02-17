#ifndef THREAD
#define THREAD

#include "base_thread.h"
#include "auto_aim.h"

class Thread: public BaseThread{
public:
    Thread();
    ~Thread();

public:
    void init(AutoAim *pAutoaim);
    void run();
private:
    AutoAim *autoaim;
};

#endif