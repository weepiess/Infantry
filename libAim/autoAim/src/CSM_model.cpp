#include "CSM_model.h"
#include <fstream>
using namespace std;
using namespace Eigen;
const float pi =3.141592;

Model::Model()
{
	T=1;//采样间隔
	a=1.0/60;//机动时间常数的倒数
	a_max=20;//目标加速度上限

	double q11,q12,q13,q22,q23,q33;
	q11=(1-exp(-2*a*T)+2*a*T+2*pow(a,3)*pow(T,3)/3-2*pow(a,2)*pow(T,2)-4*a*T*exp(-a*T))/(2*pow(a,5));
	q12=(exp(-2*a*T)+1-2*exp(-a*T)+2*a*T*exp(-a*T)-2*a*T+pow(a,2)*pow(T,2))/(2*pow(a,4));
	q13=(1-exp(-2*a*T)-2*a*T*exp(-a*T))/(2*pow(a,3));
	q22=(4*exp(-a*T)-3-exp(-2*a*T)+2*a*T)/(2*pow(a,3));
	q23=(exp(-2*a*T)+1-2*exp(-a*T))/(2*pow(a,2));
	q33=(1-exp(-2*a*T))/(2*a);

	Q<<q11,q12,q13,
	q12,q22,q23,
	q13,q23,q33;

	//状态转移矩阵
	F<<1,T,(-1+a*T+exp(-a*T))/pow(a,2),
    	0,1,(1-exp(-a*T))/a,
    	0,0,exp(-a*T);
		
	F1<<1,T,pow(T,2)/2,
		0,1,T,
		0,0,1;

	//量测矩阵
	H=MatrixXf::Zero(1,3);
	H<<1,0,0;

	//状态输入矩阵
	U=MatrixXf::Zero(3,1);
	U<<(-T+a*pow(T,2)/2+(1-exp(-a*T))/a)/a,
		T-(1-exp(-a*T))/a,
    	1-exp(-a*T);	

	for(int i=0;i<3;++i)
	{
		X[i]=MatrixXf::Zero(3,1);
		X_P[i]=MatrixXf::Zero(3,1);
	}
}

//input:观测值
//用于初始化CSM模型
void Model::CSM_init(float x)
{	
	P[0]<<pow(10,4),0,0,
    	0,25,0,
    	0,0,1;
	P_P[0]=P[0];
	
	float v=0,a=0;

	for(int i=0;i<3;++i)
	{
		X[i](1,0)=0;
		X_P[i](1,0)=0;
	}
	X[0]<<x,v,a;

	X_P[1]=F1*X[0];
}

/*
input:y——观测值，predict——预测值，k——用于计数，axis——x为0、y为1、z为2
通过卡尔曼滤波进行预测
*/
void Model::CSM_predict(float y,float& predict,int k,int axis)
{
	float ca;

	R=pow(10,4)*MatrixXf::Random(1,1);//量测噪声协方差矩阵存储器初始化
	if(R(0,0)<0) R(0,0)=-R(0,0);
	
	float re=y-(H*X_P[(k+1)%3])(0,0);//残差值
	float S=(H*P_P[k]*H.transpose()+R)(0,0);//残差的协方差矩阵
	float u=1-exp(-abs(re))/(2*S);
	
	if(X[k](2,0)>=0)
		ca=(4-pi)/pi*pow((a_max*u-X[k](2,0)),2);
	else
		ca=(4-pi)/pi*pow((a_max*u+X[k](2,0)),2);
	
	Matrix3f Q_=2*a*ca*Q;
	
	float beta=0.0005;
	a=a+beta*(1/pow((1+exp(-50*abs(re))),2)-0.25);//自适应调整后的机动频率
	
	//由更新后的机动频率，对Q,F,U进行更新
	double q11,q12,q13,q22,q23,q33;
	q11=(1-exp(-2*a*T)+2*a*T+2*pow(a,3)*pow(T,3)/3-2*pow(a,2)*pow(T,2)-4*a*T*exp(-a*T))/(2*pow(a,5));
	q12=(exp(-2*a*T)+1-2*exp(-a*T)+2*a*T*exp(-a*T)-2*a*T+pow(a,2)*pow(T,2))/(2*pow(a,4));
	q13=(1-exp(-2*a*T)-2*a*T*exp(-a*T))/(2*pow(a,3));
	q22=(4*exp(-a*T)-3-exp(-2*a*T)+2*a*T)/(2*pow(a,3));
	q23=(exp(-2*a*T)+1-2*exp(-a*T))/(2*pow(a,2));
	q33=(1-exp(-2*a*T))/(2*a);

	Q<<q11,q12,q13,
	q12,q22,q23,
	q13,q23,q33;

	F<<1,T,(-1+a*T+exp(-a*T))/pow(a,2),
    	0,1,(1-exp(-a*T))/a,
    	0,0,exp(-a*T);

	U<<(-T+a*pow(T,2)/2+(1-exp(-a*T))/a)/a,
		T-(1-exp(-a*T))/a,
    	1-exp(-a*T);

	//卡尔曼滤波方程
	P_P[(k+1)%3]=F*P[k]*F.transpose()+Q_;
	K=P_P[(k+1)%3]*H.transpose()*(H*P_P[(k+1)%3]*H.transpose()+R).inverse();
	X[(k+1)%3]=X_P[(k+1)%3]+K*(y-(H*X_P[(k+1)%3])(0,0));
	P[(k+1)%3]=(MatrixXf::Identity(3,3)-K*H)*P_P[(k+1)%3];
	X_P[(k+2)%3]=F1*X[(k+1)%3];

	predict=X_P[(k+2)%3](0,0);
}

/*
input:M1——转换前的坐标，M2——转换后的坐标，flag——0用于从相机坐标转为世界坐标，1用于从世界坐标转回相机坐标
坐标转换
*/
void Predict::trans(float pitch,float yaw,MatrixXf& M1,MatrixXf& M2,int flag)
{
	float radianP,radianY;
	radianP=pitch/180*pi;
	radianY=yaw/180*pi;

	Matrix3f A(3,3);
	A<<1,0,0,
	0,cos(radianP),-sin(radianP),
	0,sin(radianP),cos(radianP);

	Matrix3f B(3,3);
	B<<cos(radianY),0,sin(radianY),
	0,1,0,
	-sin(radianY),0,cos(radianY);

	if(flag==0)
		M2=M1*A*B;
	else 
		M2=M1*B*A;
}

/*
input:isReset——连续三帧跟丢后，再次识别到，此时重置模型，isReset为1
				如果是在三帧之内跟丢，用预测值进行补偿，isReset为0
	  pitch——绝对pitch值
	  yaw——绝对yaw值
*/
void Predict::run(int isReset,vector<float>& now,vector<float>& pred,float pitch,float yaw)
{
	//cout<<"before:"<<now[0]<<" "<<now[1]<<" "<<now[2]<<endl;
	MatrixXf M1(1,3);
	MatrixXf M2(1,3);
	float x_,y_,z_;

	M1<<now[0],now[1],now[2];
	trans(pitch,yaw,M1,M2,0);

	x_=M2(0,0);y_=M2(0,1);z_=M2(0,2);
	cout<<"prim:"<<x_<<" "<<z_<<endl;

	//savePrim(flag,x_,z_);
//-------------------------运动模型----------------------
	float x_pred,z_pred;

	if(isReset==1) 
	{
		tracking_x.CSM_init(x_);
		tracking_z.CSM_init(z_);
		times=0;
	}

	tracking_x.CSM_predict(x_, x_pred, times,0);
	tracking_z.CSM_predict(z_, z_pred, times,2);

	times=(times+1)%3;

	cout<<"pred1:"<<x_pred<<" "<<z_pred<<endl;
	//savePred(flag,x_pred,z_pred);
//-------------------------------------------------------
	M1<<x_pred,y_,z_pred;

	trans(-pitch,-yaw,M1,M2,1);
	x_pred=M2(0,0);y_=M2(0,1);z_pred=M2(0,2);
	//cout<<"pred2:"<<x_pred<<" "<<z_pred<<endl;

	pred.push_back(x_pred);
	pred.push_back(y_);
	pred.push_back(z_pred);

	last_pred[0]=tracking_x.nextframe;
	last_pred[1]=now[1];
	last_pred[2]=tracking_z.nextframe;
}

/*正常的预测(没有三帧丢失，非补偿）*/
void Predict::run(vector<float>& pred,float pitch,float yaw)
{
//-------------------------运动模型----------------------
	float x_pred,z_pred;

	tracking_x.CSM_predict(last_pred[0],x_pred,times,0);
	tracking_z.CSM_predict(last_pred[2],z_pred,times,2);

	times=(times+1)%3;

	cout<<"pred1:"<<x_pred<<" "<<z_pred<<endl;
//-------------------------------------------------------
	MatrixXf M1(1,3);
	MatrixXf M2(1,3);
	M1<<x_pred,last_pred[1],z_pred;

	trans(-pitch,-yaw,M1,M2,1);
	//cout<<"pred2:"<<M2(0,0)<<" "<<M2(0,2)<<endl;

	pred.push_back(M2(0,0));
	pred.push_back(M2(0,1));
	pred.push_back(M2(0,2));

	last_pred[0]=tracking_x.nextframe;
	last_pred[2]=tracking_z.nextframe;
}