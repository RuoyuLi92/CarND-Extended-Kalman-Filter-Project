#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
   // initialiaze the rmse vector
   VectorXd rmse(4);
   rmse << 0, 0, 0, 0;
   
   size_t sizeOfStep = estimations.size();
   
   // check validity of input vector
   if(sizeOfStep == 0)
   {
	   std::cout << "CalculateRMSE() - Error - the estimation vector size should not be zero";
   }
   
   if (sizeOfStep != ground_truth.size()) 
   {
	   std::cout << "CalculateRMSE() - Error - the estimation vector size should equal ground truth vector size";
   }
   
   // calculate the sum of suqare of errors over time
   for(size_t i = 0; i<sizeOfStep; ++i)
   {
	   VectorXd e = estimations[i] - ground_truth[i];
	   e = e.array() * e.array();
	   rmse += e;
   }
   
   // calculate mean over time step
   rmse /= sizeOfStep;
   
   //calculate suqare root for each row in the vector(px,py,vx,vy) 
   rmse = rmse.array().sqrt();
   
   return rmse;
}


MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
   
   // intitialiaze the jacobian
   MatrixXd jacobian = MatrixXd::Zero(3,4);
   
   // read from input x_state
   float px = x_state[0];
   float py = x_state[1];
   float vx = x_state[2];
   float vy = x_state[3];
   
   if (px == 0 and py == 0) 
   {
		std::cout << "CalculateJacobian() - Error - Division by Zero";
   }
   
   float sqrSum = px * px + py * py;
   float  root = sqrt(sqrSum);
   float  rootPow3 = sqrSum * root;
   
   jacobian(0,0) = px / root;
   jacobian(0,1) = py / root;
   
   jacobian(1,0) = -py / sqrSum;
   jacobian(1,1) = px / sqrSum;
   
   jacobian(2,0) = py * (vx * py - vy * px) / rootPow3;
   jacobian(2,1) = px * (px * vy - vx * py) / rootPow3;
   jacobian(2,2) = jacobian(0,0);
   jacobian(2,3) = jacobian(0,1);
   
   return jacobian;
}
