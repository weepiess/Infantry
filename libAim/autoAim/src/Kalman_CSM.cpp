#include <iostream>
#include <stdexcept>
#include "Kalman_CSM.h"
#include "emmintrin.h"
#include "tmmintrin.h"
#include <immintrin.h>
using namespace std;
const float pi =3.141592;

KalmanCSMFilter::KalmanCSMFilter(
    double dt)
  : dt(dt), initialized(false),A(3, 3),C(1,3),Q(3,3),R(1,1),P(3,3),U(3,1),
    I(3, 3), x_hat(3), x_hat_new(3)
{
  beta_csm = 0.0005;
  a=1.0/60;
  a_max=20;
  I.setIdentity();
  double q11,q12,q13,q22,q23,q33;
	q11 = (1-exp(-2*a*dt)+2*a*dt+2*pow(a,3)*pow(dt,3)/3-2*pow(a,2)*pow(dt,2)-4*a*dt*exp(-a*dt))/(2*pow(a,5));
	q12 = (exp(-2*a*dt)+1-2*exp(-a*dt)+2*a*dt*exp(-a*dt)-2*a*dt+pow(a,2)*pow(dt,2))/(2*pow(a,4));
	q13 = (1-exp(-2*a*dt)-2*a*dt*exp(-a*dt))/(2*pow(a,3));
	q22 = (4*exp(-a*dt)-3-exp(-2*a*dt)+2*a*dt)/(2*pow(a,3));
	q23 = (exp(-2*a*dt)+1-2*exp(-a*dt))/(2*pow(a,2));
	q33 = (1-exp(-2*a*dt))/(2*a);
	Q_CSM << q11,q12,q13,
	         q12,q22,q23,
  	       q13,q23,q33;
  A << 1, dt, (-1+a*dt+exp(-a*dt))/pow(a,2),
    	 0, 1,  (1-exp(-a*dt))/a,
    	 0, 0,   exp(-a*dt);
  C << 1,0,0; 
  U << (-dt+a*pow(dt,2)/2+(1-exp(-a*dt))/a)/a,
	     dt-(1-exp(-a*dt))/a,
    	 1-exp(-a*dt);
  P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;
  R << .0001;
  
}

KalmanCSMFilter::KalmanCSMFilter() {}

void KalmanCSMFilter::init(double t0, const Eigen::VectorXd& x0) {
  x_hat = x0;
  this->t0 = t0;
  t = t0;
  initialized = true;
  
}

void KalmanCSMFilter::init() {
  x_hat.setZero();
  t0 = 0;
  t = t0;
  initialized = true;
}
Eigen::Matrix3d calQCSM(float a , float dt){
  float at = a * dt;
  float t2 = exp(-at);
  __attribute__ ((aligned(16))) float op1[4] = {at, at, a, dt};  
  __attribute__ ((aligned(16))) float op2[4] = {t2, 2.0, a, dt}; 
  __attribute__ ((aligned(16))) float op3[4] = {2.0,2.0, a, dt};
  __attribute__ ((aligned(16))) float result[4]; 
  __attribute__ ((aligned(16))) float result1[4]; 
  __m128  a1,b,c,d,e,f,g;  
  a1 = _mm_load_ps(op1);
  b = _mm_load_ps(op2);
  c = _mm_load_ps(op3);
  d = _mm_mul_ps(a1,b);
  e = _mm_mul_ps(e,d);

  _mm_store_ps(result,d);
  _mm_store_ps(result1,e);
  float t1 = 1-exp(-result[1]);

  double q11,q12,q13,q22,q23,q33;
	q11=(t1+result[1]+2*result1[2]*result1[3]/3-2*result[2]*result[3]-4*result[0])/(2*pow(a,5));
	q12=(exp(-result[1])+1-2*t2+2*result[0]-result[1]+result[2]*result[3])/(2*pow(a,4));
	q13=(t1-2*result[0])/(2*result1[2]);
	q22=(4*t2-3-exp(-result[1])+result[1])/(2*result1[2]);
	q23=(exp(-result[1])+1-2*t2)/(2*result[2]);
	q33=t1/(2*a);
  Matrix3d Temp;
  Temp << q11,q12,q13,
	        q12,q22,q23,
	        q13,q23,q33;
  return Temp; 
}
void KalmanCSMFilter::update(float y) {

  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");

  x_hat_new = A * x_hat + U * x_hat(2);
  float e = y - (C * x_hat_new)(0,0);
  float S = (C * P * C.transpose() + R)(0,0);
  float u = 1-exp(-abs(e))/(2 * S); 
  float ca  = (4 - pi) / pi * pow((a_max * u - fabs(x_hat(2))),2);
  Q = 2 * a * ca * Q_CSM;
  float beta=0.0005;
  float a=a+beta*(1/pow((1+exp(-50*fabs(e))),2)-0.25);
  
  Q_CSM = calQCSM(a,dt);
  P = A * P * A.transpose() + Q;
  K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
  
  x_hat_new += K * (e);
  P = (I - K * C) * P;
  x_hat = x_hat_new;                                                                                                                                    
  t += dt;
}

void KalmanCSMFilter::update(float y, double dt, const Eigen::MatrixXd A) {

  this->A = A;
  this->dt = dt;
  update(y);
}