#ifndef AUTO_TRN_H
#define AUTO_TRN_H
#include "base_thread.h"
#include "queue"
#include <list>
#include "Fft.h"
#include <opencv2/opencv.hpp>
#include "auto_aim.h"
#include "atomic"
class AutoTRN: public BaseThread{
    public:
    AutoTRN();
    ~AutoTRN();

    public:
    void run();
    void init();
    void setData(float yaw_data);
    void getCondition(uchar &condition);
    void stop();
    void resume();
    private:
    void dataJudger();

    private:
        typedef enum: uchar{
        CONDITION_NOR,
        CONDITION_ABN,
        CONDITION_NOR_WIGGLE,
        CONDITION_FAST_WIGGLE,
    }cod;

    private:
    bool flag;
    bool data_update = false;
    uchar curr_condition;
    FFt fft;
    pthread_cond_t  cond = PTHREAD_COND_INITIALIZER;
    pthread_mutex_t controlMutex = PTHREAD_MUTEX_INITIALIZER; 
    AutoAim *autoaim;
    private:
    int num = 0;      //一个过渡值，对于同方向变化量大的数据，num_x越大 
    int old_flags = 0;   //表示第n-2个数据到第n-1个数据的变化趋势，加为1，减为0 
    int new_flags = 0; 
    float curr_data = 0;  //表示第n-1个数据到第n个数据的变化趋势，加为1，减为0
    float old_num = 0;     //第n-1次的答案 
    int Threshold_min = 8;
    int Threshold_max = 20;
    list<float> numbase;
    double result[64 * 2] = { 0 };
    float scope=1.5;
    int badnum = 0;
    int key = 0;
    float k =0.2;

    public:

};
#endif