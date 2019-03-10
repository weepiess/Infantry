#include "autotrn.h"

AutoTRN::AutoTRN(){}
AutoTRN::~AutoTRN(){}

void AutoTRN::init(){
    start();
}

void AutoTRN::setData(float yaw_dara){
    pthread_mutex_lock(&controlMutex);
    curr_data = yaw_data;
    pthread_mutex_unlock(&controlMutex);
    data_update = true;
}
void AutoTRN::stop(){
    pthread_mutex_lock(&controlMutex);
    flag = false;
    pthread_mutex_unlock(&controlMutex);
}
void AutoTRN::resume(){
    pthread_mutex_lock(&controlMutex);
    pthread_cond_signal(&cond);
    pthread_mutex_unlock(&controlMutex);
}

void AutoTRN::dataJudger(){
    float sum = 0;
    float mean = 100;
    float var = 100;
    float var_sum = 0;
    if(data_update){
        numbase.push_back(curr_data); //将更新数据存入list
        if(numbase.size()>=64){               //达到预设开始傅里叶变换
            int k=0;
            for (list<float>::iterator it = numbase.begin(); it != numbase.end(); ++it){ //遍历list
                sum = sum + *it;    //求和
                fft.x[k].real = *it;   //输入到实部
                fft.x[k].img = 0;      //虚部预设为0
                k++;
            }
            mean = sum/float(numbase.size());       //求均值
            for(int i=0; i < numbase.size(); i++){
                var_sum = pow(fft.x[i].real - mean, 2) + var_sum; //平方差
            }
            var = sqrt(var_sum/float(numbase.size() -1 ));      //方差
            fft.initW(numbase.size());                  //傅里叶变换初始化
            fft.fftx();                                 //开始傅里叶变换
            for (int i = 0; i < numbase.size(); i++)
            {
                result[i] = sqrt(fft.x[i].real*fft.x[i].real + fft.x[i].img*fft.x[i].img); //每个频率对应的幅值
                if(i > numbase.size()/2){
                    if(result[i]/numbase.size()*2> scope)  //高频信号幅值超过阈值
                        badnum++;
                }
            }
            numbase.pop_front(); //弹出首项
        }
    
        if(badnum>10){ //高频情况
            if(mean < 1 && var >3){ //扭腰均值较小
                pthread_mutex_lock(&controlMutex);
                curr_condition = cod::CONDITION_NOR_WIGGLE;
                pthread_mutex_unlock(&controlMutex);
                if(var > 5){
                    pthread_mutex_lock(&controlMutex);
                    curr_condition = cod::CONDITION_FAST_WIGGLE;
                    pthread_mutex_unlock(&controlMutex);
                }
            }
            else{
                pthread_mutex_lock(&controlMutex);
                curr_condition = cod::CONDITION_ABN;
                pthread_mutex_unlock(&controlMutex);
            }
        }
        else{
            pthread_mutex_lock(&controlMutex);
            curr_condition =  cod::CONDITION_NOR;
            pthread_mutex_unlock(&controlMutex);
        }
        badnum = 0;
        data_update = false;
    }   
}

void AutoTRN::getCondition(uchar &condition){
    pthread_mutex_lock(&controlMutex);
    condition =  curr_condition;
    pthread_mutex_unlock(&controlMutex);
}

void AutoTRN::run(){
    while(true){
        pthread_mutex_lock(&controlMutex);
        while(is_target_found){
            pthread_cond_wait(&cond, &controlMutex);
        }
        pthread_mutex_unlock(&controlMutex);
        dataJudger();
    }
}