
////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC ROBOMASTER2019      Model Code for robot
///ALL RIGHTS RESERVED
///@file:Kalman_CSM.h
///@brief: kalman的Eigen实现,用于CSM模型的实现及修改,但是暂时还并没有作用
/// ctrl+c ctrl+v 网上代码
///@vesion 1.0
///@author: pc
///@email: 694977655@qq.com
///@date: 18-12-9
///修订历史：
////////////////////////////////////////////////////////////////////////////////
#ifndef KALMAN_CSM_H
#define KALMAN_CSM_H
#include <eigen3/Eigen/Dense>
using namespace Eigen;
#pragma once

class KalmanCSMFilter {

public:

  /**
  * Create a Kalman filter with the specified matrices.
  *   A - System dynamics matrix
  *   C - Output matrix
  *   Q - Process noise covariance
  *   R - Measurement noise covariance
  *   P - Estimate error covariance
  */
  KalmanCSMFilter(
      double dt
  );

  /**
  * Create a blank estimator.
  */
  KalmanCSMFilter();

  /**
  * Initialize the filter with initial states as zero.
  */
  void init();

  /**
  * Initialize the filter with a guess for initial states.
  */
  void init(double t0, const VectorXd& x0);

  /**
  * Update the estimated state based on measured values. The
  * time step is assumed to remain constant.
  */
  void update(float y);

  /**
  * Update the estimated state based on measured values,
  * using the given time step and dynamics matrix.
  */
  void update(float y, double dt, const MatrixXd A);

  /**
  * Return the current state and time.
  */
  VectorXd state() { return x_hat; };
  double time() { return t; };

private:

  // Matrices for computation
  MatrixXd A;
  MatrixXd C;
  MatrixXd Q;
  MatrixXd R;
  MatrixXd P;
  MatrixXd K, P0;
  MatrixXd U;
  Matrix3d Q_CSM;

  // System dimensions
  int m, n;

  // Initial and current time
  double t0, t;

  // Discrete time step
  double dt;
  
  //机动时间常数的倒数
  double a;

  //自适应方法系数
  double beta_csm;

  //目标加速度上限
  double a_max ;

  // Is the filter initialized?
  bool initialized;

  // n-size identity
  Eigen::MatrixXd I;

  // Estimated states
  Eigen::VectorXd x_hat, x_hat_new;
};
#endif
//example
// #include <iostream>
// #include <vector>
// #include <Eigen/Dense>

// #include "kalman.hpp"

// int main(int argc, char* argv[]) {

//   int n = 3; // Number of states
//   int m = 1; // Number of measurements

//   double dt = 1.0/30; // Time step

//   Eigen::MatrixXd A(n, n); // System dynamics matrix
//   Eigen::MatrixXd C(m, n); // Output matrix
//   Eigen::MatrixXd Q(n, n); // Process noise covariance
//   Eigen::MatrixXd R(m, m); // Measurement noise covariance
//   Eigen::MatrixXd P(n, n); // Estimate error covariance

//   // Discrete LTI projectile motion, measuring position only
//   A << 1, dt, 0, 0, 1, dt, 0, 0, 1;
//   C << 1, 0, 0;

//   // Reasonable covariance matrices
//   Q << .05, .05, .0, .05, .05, .0, .0, .0, .0;
//   R << 5;
//   P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;

//   std::cout << "A: \n" << A << std::endl;
//   std::cout << "C: \n" << C << std::endl;
//   std::cout << "Q: \n" << Q << std::endl;
//   std::cout << "R: \n" << R << std::endl;
//   std::cout << "P: \n" << P << std::endl;

//   // Construct the filter
//   KalmanFilter kf(A, C, Q, R, P);

//   // List of noisy position measurements (y)
//   std::vector<double> measurements = {
//       1.04202710058, 1.10726790452, 1.2913511148, 1.48485250951, 1.72825901034,
//       1.74216489744, 2.11672039768, 2.14529225112, 2.16029641405, 2.21269371128,
//       2.57709350237, 2.6682215744, 2.51641839428, 2.76034056782, 2.88131780617,
//       2.88373786518, 2.9448468727, 2.82866600131, 3.0006601946, 3.12920591669,
//       2.858361783, 2.83808170354, 2.68975330958, 2.66533185589, 2.81613499531,
//       2.81003612051, 2.88321849354, 2.69789264832, 2.4342229249, 2.23464791825,
//       2.30278776224, 2.02069770395, 1.94393985809, 1.82498398739, 1.52526230354,
//       1.86967808173, 1.18073207847, 1.10729605087, 0.916168349913, 0.678547664519,
//       0.562381751596, 0.355468474885, -0.155607486619, -0.287198661013, -0.602973173813
//   };

//   // Best guess of initial states
//   Eigen::VectorXd x0(n);
//   x0 << measurements[0], 0, -9.81;
//   kf.init(x0);

//   // Feed measurements into filter, output estimated states
//   double t = 0;
//   Eigen::VectorXd y(m);
//   std::cout << "t = " << t << ", " << "x_hat[0]: " << kf.state().transpose() << std::endl;
//   for(int i = 0; i < measurements.size(); i++) {
//     t += dt;
//     y << measurements[i];
//     kf.update(y);
//     std::cout << "t = " << t << ", " << "y[" << i << "] = " << y.transpose()
//         << ", x_hat[" << i << "] = " << kf.state().transpose() << std::endl;
//   }

//   return 0;
// }