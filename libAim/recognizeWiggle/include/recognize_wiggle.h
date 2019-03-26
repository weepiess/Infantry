#ifndef RECOGNIZE_WIGGLE_H
#define RECOGNIZE_WIGGLE_H
#include "base_thread.h"
#include "queue"
#include <list>
#include "fast_fourier_transform.h"
#include <opencv2/opencv.hpp>
#include "atomic"
class RecognizeWiggle: public BaseThread{
    public:
    RecognizeWiggle();
    ~RecognizeWiggle();

    public:
    void run();
    void init();
    void setData(float yaw_data);
    void getCondition(uchar &condition);
    void stop();
    void resume();
    
    private:
    void dataJudger();

    public:
        typedef enum: uchar{
        CONDITION_NOR,
        CONDITION_ABN,
        CONDITION_NOR_WIGGLE,
        CONDITION_FAST_WIGGLE,
    }EnemyConditon;


    private:
    FFt fft_;
    pthread_cond_t  cond_ = PTHREAD_COND_INITIALIZER;
    pthread_mutex_t control_mutex_ = PTHREAD_MUTEX_INITIALIZER; 

    private:
    bool flag_;
    bool data_update_ = false;
    uchar current_condition_=1;
    float curr_data_ = 0;
    list<float> numbase_;
    double result_[64 * 2] = { 0 };
    float scope_=1.5;
    int badnum_ = 0;
    float k_ =0.2;
};
#endif