#ifndef AUTO_PTZ_H
#define AUTO_PTZ_H
#include "base_thread.h"

class AutoPTZ: public BaseThread{

    public:
        AutoPTZ();
        ~AutoPTZ();
    public:
        typedef enum: unsigned char{
            FAST,
            MEDIUM,
            SLOW
        }ScanSpeed;

        typedef enum: unsigned char{
            CLOCKWISE,
            COUNTERCLOCKWISE
        }ScanDirection;

        void init();
        float getAngle();
        void setCurrentAngel(float yaw);
        void setScanSpeed(ScanSpeed speed);
        void setScanDirection(ScanDirection direction);
        void isTargetFind(bool flag);
        void threadResume();  
        void autoFunc();
    private:
        void run();

    private:
        bool is_target_found_;
        ScanDirection scan_direction_;
        ScanSpeed speed_level_;
        float current_yaw_;
        float yaw_tmp_;
        pthread_cond_t  cond_ = PTHREAD_COND_INITIALIZER;
        pthread_mutex_t control_mutex_ = PTHREAD_MUTEX_INITIALIZER; //互斥变量锁    
};

#endif