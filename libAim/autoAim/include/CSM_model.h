#ifndef AIM_MODEL_H_
#define AIM_MODEL_H_

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <vector>

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

	float velocity,acceleration;

	float getMin(float a,float b,float c);
	float getMax(float a,float b,float c);
	float amend(float reality,int k,int axis);
public:
	Model();

	float nextframe;

	void CSM_init(float x0);
	float CSM_predict(float y,int k,int axis);
};

class Predict
{
private:
	void trans(float pitch,float yaw,Eigen::MatrixXf& M1,Eigen::MatrixXf& M2,int flag);

	Model tracking_x,tracking_z;
	int times=0;
	float last_pred[3];

public:
	void run(int flag,std::vector<float>& now,std::vector<float>& pred,float pitch,float yaw);
	void run(std::vector<float>& pred,float pitch,float yaw);
};

#endif
