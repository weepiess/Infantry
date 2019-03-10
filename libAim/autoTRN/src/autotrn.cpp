#include "autotrn.h"

AutoTRN::AutoTRN(){}
AutoTRN::~AutoTRN(){}

void AutoTRN::init(AutoAim *pAutoaim){
    pthread_mutex_lock(&controlMutex);
    autoaim = pAutoaim;
    pthread_mutex_unlock(&controlMutex);
    start();
}

void AutoTRN::dataJudger(){
    pthread_mutex_lock(&controlMutex);
    float sum = 0;
    float mean = 100;
    float var = 100;
    float var_sum = 0;
    pthread_mutex_unlock(&controlMutex);
    if(autoaim->new_number != autoaim->old_number){
        pthread_mutex_lock(&controlMutex);
        numbase.push_back(autoaim->new_number); //将更新数据存入list
        pthread_mutex_unlock(&controlMutex);
        if(numbase.size()>=64){                                     //达到预设开始傅里叶变换
            pthread_mutex_lock(&controlMutex);
            int k=0;
            pthread_mutex_unlock(&controlMutex);
            for (list<float>::iterator it = numbase.begin(); it != numbase.end(); ++it){ //遍历list
                pthread_mutex_lock(&controlMutex);
                sum = sum + *it;    //求和
                fft.x[k].real = *it;   //输入到实部
                fft.x[k].img = 0;      //虚部预设为0
                k++;
                pthread_mutex_unlock(&controlMutex);
            }
            pthread_mutex_lock(&controlMutex);
            mean = sum/float(numbase.size());       //求均值
            pthread_mutex_unlock(&controlMutex);
            for(int i=0; i < numbase.size(); i++){
                pthread_mutex_lock(&controlMutex);
                var_sum = pow(fft.x[i].real - mean, 2) + var_sum; //平方差
                pthread_mutex_unlock(&controlMutex);
            }
            pthread_mutex_lock(&controlMutex);
            var = sqrt(var_sum/float(numbase.size() -1 ));      //方差
            fft.initW(numbase.size());                  //傅里叶变换初始化
            fft.fftx();                                                //开始傅里叶变换
            pthread_mutex_unlock(&controlMutex);
            for (int i = 0; i < numbase.size(); i++)
            {
                pthread_mutex_lock(&controlMutex);
                // ofstream outfile("/home/weepies/output.txt", ios::app);
                result[i] = sqrt(fft.x[i].real*fft.x[i].real + fft.x[i].img*fft.x[i].img); //每个频率对应的幅值
                // cout<<result[i]<<"    *** res i"<<endl;
                // cout << setprecision(2) << result[i]/numbase.size()*2 << " ************************"<<endl;;
                // outfile<< result[i]/numbase.size()*2<<endl;
                pthread_mutex_unlock(&controlMutex);
                if(i > numbase.size()/2){
                    if(result[i]/numbase.size()*2> scope)  //高频信号幅值超过阈值
                        pthread_mutex_lock(&controlMutex);
                        badnum++;
                        pthread_mutex_unlock(&controlMutex);
                }
            }
            pthread_mutex_lock(&controlMutex);
            numbase.pop_front(); //弹出首项
            pthread_mutex_unlock(&controlMutex);
        }
        cout<<badnum<<"  badnum                                       !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
        if(badnum>10){ //高频情况
            pthread_mutex_lock(&controlMutex);
            // ofstream outfile2("/home/weepies/outputt.txt", ios::ate);
            // outfile2<<mean<<"   mean "<<var<<"  var "<<endl;
            pthread_mutex_unlock(&controlMutex);
            if(mean < 1 && var >3){ //扭腰均值较小
                pthread_mutex_lock(&controlMutex);
                autoaim->condition = cod::CONDITION_NOR_WIGGLE;
                pthread_mutex_unlock(&controlMutex);
                if(var > 5){
                    pthread_mutex_lock(&controlMutex);
                    autoaim->condition = cod::CONDITION_FAST_WIGGLE;
                    pthread_mutex_unlock(&controlMutex);
                }
            }
            else{
                pthread_mutex_lock(&controlMutex);
                autoaim->condition = cod::CONDITION_ABN;
                pthread_mutex_unlock(&controlMutex);
            }
        }
        else{
            pthread_mutex_lock(&controlMutex);
            autoaim->condition =  cod::CONDITION_NOR;
            pthread_mutex_unlock(&controlMutex);
        }
        pthread_mutex_lock(&controlMutex);
        badnum = 0;
        autoaim->old_number = autoaim->new_number; //更新
        pthread_mutex_unlock(&controlMutex);
    }   
}

void AutoTRN::run(){
    while(true){
        dataJudger();
    }
}