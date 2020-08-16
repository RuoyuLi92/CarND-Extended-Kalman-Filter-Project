#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
   x_ = F_ * x_;
   P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
   VectorXd y = z - H_ * x_;
   MatrixXd S = H_ * P_ * H_.transpose() + R_;
   MatrixXd K = P_ * H_.transpose() * S.inverse();
   MatrixXd I = MatrixXd::Identity(P_.rows(), P_.rows());
   
   x_ += K * y;
   P_ = (I - K * H_) * P_;
   
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
   
   //VectorXd y = z - H_ * x_;
   float px = x_(0);
   float py = x_(1);
   float vx = x_(2);
   float vy = x_(3);
   
   float rho = sqrt(px * px + py * py);
   //float phi = atan(py / px);               // to use atan() instead of atan2() uncomment this line
   float phi = atan2(py, px);
   std::cout << "phi_est = " <<phi<<std::endl;
   
   float drho = (px * vx + py * vy)/rho;
   
   VectorXd hx(3);
   hx << rho, phi, drho;
   
   VectorXd y = z - hx;
   std::cout << "phi_mess = " <<z(1)<<std::endl;
   
   
   // To use atan() instead of atan2() uncomment these two line
   /*
   while(y(1) > (PI/2)) y(1) -= PI;
   while(y(1) < -1*(PI/2)) y(1) += PI;
   */
   
   
   // To use atan2() comment these two line
   
   while(y(1) > PI) y(1) -= 2*PI;
   while(y(1) < -PI) y(1) += 2*PI;
   
   
   std::cout << "phi_corrected = " <<z(1) - y(1)<<std::endl;
   
   MatrixXd S = H_ * P_ * H_.transpose() + R_;
   MatrixXd K = P_ * H_.transpose() * S.inverse();
   MatrixXd I = MatrixXd::Identity(P_.rows(), P_.rows());
   
   x_ += K * y;
   P_ = (I - K * H_) * P_;
}
