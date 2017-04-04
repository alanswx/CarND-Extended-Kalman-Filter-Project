#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

/*

This function is used to factor out the code that is the same in
the laser and radar case.

*/
void KalmanFilter::UpdateInternal(const VectorXd &z,const VectorXd &y)
{
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K =  P_ * Ht * Si;
	MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());

		//new state
	x_ = x_ + (K * y);
	P_ = (I - K * H_) * P_;
}


void KalmanFilter::Update(const VectorXd &z) {
  	VectorXd y = z - H_*x_;
	UpdateInternal(z,y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    double px = x_[0];
    double py = x_[1];
    double vx = x_[2];
    double vy = x_[3];

	double pxpy=sqrt(px*px+py*py);
  	if (pxpy<=0.001)
  	{
  	  std::cout << "ERROR" << std::endl;
  	  pxpy=0.00001;
  	}
    VectorXd hxprime = VectorXd(3);
	hxprime(0) = pxpy;
	hxprime(1) = atan2(py,px);
	hxprime(2) = (px*vx+py*vy)/pxpy;
	
	VectorXd y = z - hxprime;
	// need to check to see if we blow out the -pi/pi
	if (y(1) > M_PI || y(1) < -M_PI){
		std::cout << "ERROR outside pi/-pi range " << std::endl;
		std::cout << y(1) << std::endl;
		int res = floor(y(1)/M_PI);
		y(1)=y(1)/res;
		std::cout << y(1) << std::endl;
	}	
	
	UpdateInternal(z,y);

  
}


