#ifndef THREAD
#define THREAD

#include "base_thread.h"
#include "auto_aim.h"

class NYdector: public BaseThread{
public:
    NYdector();
    ~NYdector();

public:
    void init(AutoAim *pAutoaim);
    void run();
private:
    AutoAim *autoaim;
};

#endif