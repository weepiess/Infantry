#include "kalman_filter_by_opencv.h"

Kalman_filter::Kalman_filter(){}
Kalman_filter::~Kalman_filter(){}

void Kalman_filter::init(int dynamParams,int g_measure_params ,int g_control_params){
    Kf.init(dynamParams,g_measure_params,g_control_params);
    Kf.measurementMatrix = g_measurement_matrix;
    Kf.measurementNoiseCov = g_measurement_noise_cov;
    Kf.processNoiseCov = g_process_noise_cov;
    Kf.controlMatrix = g_control_matrix;
    setIdentity(Kf.errorCovPost, Scalar::all(0.001)); 
    
}
void Kalman_filter::setSamplingtime(float dt){
   g_transition_matrix=(Mat_<float>(2, 2) <<   
            1,dt,   
            0,1);
    Kf.transitionMatrix = g_transition_matrix;
}

void Kalman_filter::setStatepost(Mat statePost){
    Kf.statePost=statePost;
}

Mat Kalman_filter::predict(){
    
    return Kf.predict();
}

void Kalman_filter::correct(Mat measurement){
    Kf.correct(measurement);
}