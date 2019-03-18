#ifndef KALMAN_FILTER_BY_OPENCV_H
#define KALMAN_FILTER_BY_OPENCV_H

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

class Kalman_filter{
public:
    Kalman_filter();
    ~Kalman_filter();

public:
    void init(int dynamParams,int measureParams, int controlParams);
    void setSamplingtime(float dt);
    void setStatepost(Mat statePost);
    Mat predict();
    void correct(Mat measurement);
    
public:
    //int dynamParams;
    int g_measure_params;
    int g_control_params;
    //Mat transitionMatrix;
    //Mat g_state_pre;           //!< predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k)  
             //!< corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))  
    Mat g_transition_matrix;   //!< state transition matrix (A)  
    Mat g_control_matrix;      //!< control matrix (B) (not used if there is no control)  
    Mat g_measurement_matrix;  //!< measurement matrix (H)  
    Mat g_process_noise_cov;    //!< process noise covariance matrix (Q)  
    Mat g_measurement_noise_cov;//!< measurement noise covariance matrix (R)  
    // Mat errorCovPre;        //!< priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)*/  
    // Mat gain;               //!< Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)  
    // Mat errorCovPost;

private:
    KalmanFilter Kf;
};

#endif
/*example of using kalman_filter_by_opencv CV modle
Kalman_filter kf;
Mat measurement = Mat::zeros(4, 1, CV_32F); 
kf.transitionMatrix=(Mat_<float>(4, 4) <<   
            1,0,dt,0,   
            0,1,0,dt,   
            0,0,1,0,   
            0,0,0,1 );
kf.measurementMatrix=(Mat_<float>(4, 4) <<   
            1,0,0,0,   
            0,1,0,0,   
            0,0,1,0,   
            0,0,0,1 );  
kf.measurementNoiseCov(Mat_<float>(4, 4) <<   
            2000,0,0,0,   
            0,2000,0,0,   
            0,0,10000,0,   
            0,0,0,10000 );
kf.init(4,1e-5,1e-1,1/50);
measurement.at<float>(0)= (float)x;  
measurement.at<float>(1) = (float)y;  
measurement.at<float>(2)= (float)vx;  
measurement.at<float>(3) = (float)vy;  
psoestate=kf.predict();
kf.correct(measurement)
*/