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
  /**
  TODO:
    * predict the state
  */
     std::cout << "KalmanFilter::Predict " << std::endl;

  	//VectorXd u = VectorXd(2);
	//u << 0, 0;

  	//x_ = F_ * x_ + u;
  	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}


void KalmanFilter::UpdateInternal(const VectorXd &z,const VectorXd &y)
{
       std::cout << "KalmanFilter::UpdateInternal " << std::endl;
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
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
       std::cout << "KalmanFilter::Update " << std::endl;
  	VectorXd y = z - H_*x_;
	UpdateInternal(z,y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  
    double px = x_[0];
    double py = x_[1];
    double vx = x_[2];
    double vy = x_[3];

	double pxpy=sqrt(px*px+py*py);
  
    VectorXd hxprime = VectorXd(3);
	hxprime(0) = pxpy;
	hxprime(1) = atan2(py,px);
	hxprime(2) = (px*vx+py*vy)/pxpy;
	
	VectorXd y = z - hxprime;
	// need to check to see if we blow out the -pi/pi
	if (y(1) > M_PI || y(1) < -M_PI){
		std::cout << "outside pi/-pi range " << std::endl;
		std::cout << y(1) << std::endl;
		int res = floor(y(1)/M_PI);
		y(1)=y(1)/res;
		std::cout << y(1) << std::endl;
	}	
	
	UpdateInternal(z,y);

  
}


