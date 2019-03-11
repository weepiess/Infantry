#include"aim_predict.h"
using namespace cv;
Aim_predict::Aim_predict(){}
Aim_predict::~Aim_predict(){}

void Aim_predict::model_init(){
    mKf.measurementMatrix=(Mat_<float>(1, 2) <<   
            1,0);  
    mKf.measurementNoiseCov=(Mat_<float>(1, 1) <<   
            0.0001);

    mKf.processNoiseCov=(Mat_<float>(2,2) <<
			10000,0,
			0,1); 
    mKf.init(2,1,0);
}

void Aim_predict::reset_kf_statepost(Mat statepost){
    mKf.set_statepost(statepost);
}

Mat Aim_predict::predict(Mat measurement, float dt){
    mKf.set_samplingtime(dt);
    mKf.correct(measurement);
    return mKf.predict();
}
void Aim_predict::clear(){
    mAngle_Velocity.clear();
    mYaw.clear();
    mPredict_Angle.clear();
}
bool Aim_predict::shoot_logic(float initYaw, float angel_velocity, float predict_angle, uchar mode){
    if(mAngle_Velocity.size()<2){
        mAngle_Velocity.push_back(angel_velocity);
        mYaw.push_back(initYaw);
        mPredict_Angle.push_back(predict_angle);
        if(mPredict_Angle.size()==2){
            float iYaw_mean=std::accumulate(std::begin(mYaw),std::end(mYaw),0.0)/2;
            float v_a_mean=std::accumulate(std::begin(mAngle_Velocity),std::end(mAngle_Velocity),0.0)/2;
            float pred_angle_mean=std::accumulate(std::begin(mPredict_Angle),std::end(mPredict_Angle),0.0)/2;
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
