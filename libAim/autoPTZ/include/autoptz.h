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
        void AutoFunc();
    private:
        void run();

    private:
        bool is_target_found;
        ScanDirection scan_direction;
        ScanSpeed speed_level;
        float current_yaw;
        float yaw_tmp;
        float scan_control;
        pthread_cond_t  cond = PTHREAD_COND_INITIALIZER;
        pthread_mutex_t controlMutex = PTHREAD_MUTEX_INITIALIZER; //互斥变量锁    
};

#endif