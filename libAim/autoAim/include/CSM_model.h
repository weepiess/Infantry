#ifndef AIM_MODEL_H_
#define AIM_MODEL_H_

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>

class Model
{
private:
	float T,a,a_max;
	Eigen::Matrix3f Q;
	Eigen::Matrix3f F;
	Eigen::Matrix3f F1;
	Eigen::MatrixXf U;

	Eigen::Matrix3f P[3],P_P[3];
	Eigen::MatrixXf X[3],X_P[3];

	Eigen::MatrixXf R;
	Eigen::MatrixXf K;
	Eigen::MatrixXf H;
public:
	Model();

	float nextframe;

	void CSM_init(float x0);
	void CSM_predict(float y,float& predict,int k,int axis);
};

class Predict
{
private:
	Model tracking_x,tracking_z;
	void trans(float pitch,float yaw,Eigen::MatrixXf& M1,Eigen::MatrixXf& M2,int flag);
	int times=0;
	float last_pred[3];

public:
	void run(int flag,std::vector<float>& now,std::vector<float>& pred,float pitch,float yaw);
	void run(std::vector<float>& pred,float pitch,float yaw);
};

#endif