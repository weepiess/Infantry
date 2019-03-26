#include"aim_predict.h"
using namespace cv;
Aim_predict::Aim_predict(){}
Aim_predict::~Aim_predict(){}

void Aim_predict::modelInit(){
    kalman_fileter_.g_measurement_matrix=(Mat_<float>(1, 2) <<   
            1,0);  
    kalman_fileter_.g_measurement_noise_cov=(Mat_<float>(1, 1) <<   
            0.0001);

    kalman_fileter_.g_process_noise_cov=(Mat_<float>(2,2) <<
			10000,0,
			0,1); 
    kalman_fileter_.init(2,1,0);
}

void Aim_predict::resetKfStatepost(Mat statepost){
    kalman_fileter_.setStatepost(statepost);
}

Mat Aim_predict::predict(Mat measurement, float dt){
    kalman_fileter_.setSamplingtime(dt);
    kalman_fileter_.correct(measurement);
    return kalman_fileter_.predict();
}
void Aim_predict::clear(){
    angle_velocity_.clear();
    yaw_.clear();
    predict_angle_.clear();
}
bool Aim_predict::shootLogic(float initYaw, float angel_velocity, float predict_angle, uchar mode){
    if(angle_velocity_.size()<2){
        angle_velocity_.push_back(angel_velocity);
        yaw_.push_back(initYaw);
        predict_angle_.push_back(predict_angle);
        if(predict_angle_.size()==2){
            float iYaw_mean=std::accumulate(std::begin(yaw_),std::end(yaw_),0.0)/2;
            float v_a_mean=std::accumulate(std::begin(angle_velocity_),std::end(angle_velocity_),0.0)/2;
            float pred_angle_mean=std::accumulate(std::begin(predict_angle_),std::end(predict_angle_),0.0)/2;
            clear();
            if(int(mode) == 2 || int(mode) ==3){
                if(fabs(iYaw_mean)>0.45&&fabs(v_a_mean)>0.004){
                if(v_a_mean<-0.02&&iYaw_mean>0.2&&pred_angle_mean<0) {cout<<"*******"<<endl; return true;}
                else if(v_a_mean>0.02&&iYaw_mean<-0.2 && pred_angle_mean>0) {cout<<"======="<<endl; return true;}
                else {cout<<"qqqq"<<endl; return false;}
            }
            else if(fabs(iYaw_mean)<=0.45&&fabs(v_a_mean)<=0.009) {cout<<"++++++++"<<endl; return true;}
            else {cout<<"pppppppp"<<endl; return false;}
            }
            else{
                if(fabs(iYaw_mean)>0.45&&fabs(v_a_mean)>0.004){
                    if(v_a_mean<-0.02&&iYaw_mean>0.2&&pred_angle_mean<0) {cout<<"*******"<<endl; return true;}
                    else if(v_a_mean>0.02&&iYaw_mean<-0.2 && pred_angle_mean>0) {cout<<"======="<<endl; return true;}
                    else {cout<<"qqqq"<<endl; return false;}
                }
                else if(fabs(iYaw_mean)<=0.45&&fabs(v_a_mean)<=0.009) {cout<<"++++++++"<<endl; return true;}
                else {cout<<"pppppppp"<<endl; return false;}
            }
        }
    }else{
        clear();
        return false;
    }
}
