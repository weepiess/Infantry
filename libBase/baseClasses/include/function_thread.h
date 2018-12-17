//用于在特定时候开启线程，运行类中的某个函数
#ifndef FUNTION_THREAD
#define FUNTION_THREAD

#include <thread>
#include "base_thread.h"
#include <functional>

class FunctionThread: public BaseThread{
public:
    FunctionThread();
    virtual ~FunctionThread();

public:
    void init(std::function<void()> function_);

private:
    virtual void run() override;

    //需要运行的函数
    std::function<void()> function;
};

#endif