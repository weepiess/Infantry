#include "recognize_wiggle.h"

RecognizeWiggle::RecognizeWiggle(){}
RecognizeWiggle::~RecognizeWiggle(){}

void RecognizeWiggle::init(){
    start();
}

void RecognizeWiggle::setData(float yaw_data){
    pthread_mutex_lock(&control_mutex_);
    curr_data_ = yaw_data;
    pthread_mutex_unlock(&control_mutex_);
    data_update_ = true;
}
void RecognizeWiggle::stop(){
    pthread_mutex_lock(&control_mutex_);
    flag_ = false;
    pthread_mutex_unlock(&control_mutex_);
}
void RecognizeWiggle::resume(){
    pthread_mutex_lock(&control_mutex_);
    pthread_cond_signal(&cond_);
    pthread_mutex_unlock(&control_mutex_);
}

void RecognizeWiggle::dataJudger(){
    float sum = 0;
    float mean = 100;
    float var = 100;
    float var_sum = 0;
    if(data_update_){
        numbase_.push_back(curr_data_); //将更新数据存入list
        if(numbase_.size()>=64){               //达到预设开始傅里叶变换
            int k_=0;
            for (list<float>::iterator it = numbase_.begin(); it != numbase_.end(); ++it){ //遍历list
                sum = sum + *it;    //求和
                fft_.x[k_].real = *it;   //输入到实部
                fft_.x[k_].img = 0;      //虚部预设为0
                k_++;
            }
            mean = sum/float(numbase_.size());       //求均值
            for(int i=0; i < numbase_.size(); i++){
                var_sum = pow(fft_.x[i].real - mean, 2) + var_sum; //平方差
            }
            var = sqrt(var_sum/float(numbase_.size() -1 ));      //方差
            fft_.initW(numbase_.size());                  //傅里叶变换初始化
            fft_.fftx();                                 //开始傅里叶变换
            for (int i = 0; i < numbase_.size(); i++)
            {
                result_[i] = sqrt(fft_.x[i].real*fft_.x[i].real + fft_.x[i].img*fft_.x[i].img); //每个频率对应的幅值
                if(i > numbase_.size()/2){
                    if(result_[i]/numbase_.size()*2> scope_)  //高频信号幅值超过阈值
                        badnum_++;
                }
            }
            numbase_.pop_front(); //弹出首项
        }
    
        if(badnum_ > 10){ //高频情况
            if(mean < 1 && var > 3){ //扭腰均值较小
                pthread_mutex_lock(&control_mutex_);
                current_condition_ = EnemyConditon::CONDITION_NOR_WIGGLE;
                pthread_mutex_unlock(&control_mutex_);
                if(var > 5){
                    pthread_mutex_lock(&control_mutex_);
                    current_condition_ = EnemyConditon::CONDITION_FAST_WIGGLE;
                    pthread_mutex_unlock(&control_mutex_);
                }
            }
            else{
                pthread_mutex_lock(&control_mutex_);
                current_condition_ = EnemyConditon::CONDITION_ABN;
                pthread_mutex_unlock(&control_mutex_);
            }
        }
        else{
            pthread_mutex_lock(&control_mutex_);
            current_condition_ =  EnemyConditon::CONDITION_NOR;
            pthread_mutex_unlock(&control_mutex_);
        }
        badnum_ = 0;
        data_update_ = false;
    }   
}

void RecognizeWiggle::getCondition(uchar &condition){
    pthread_mutex_lock(&control_mutex_);
    condition =  current_condition_;
    pthread_mutex_unlock(&control_mutex_);
}

void RecognizeWiggle::run(){
    while(true){
        pthread_mutex_lock(&control_mutex_);
        while(flag_){
            pthread_cond_wait(&cond_, &control_mutex_);
        }
        pthread_mutex_unlock(&control_mutex_);
        dataJudger();
    }
}