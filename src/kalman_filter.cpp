#include "kalman_filter.h"
#include "math.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  x_ = F_ * x_;
  P_ = F_ * P_ *F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;	
  VectorXd y = z - z_pred;	
  MatrixXd Ht = H_.transpose();	
  MatrixXd S = H_ * P_ * Ht + R_;	
  MatrixXd Si = S.inverse();	
  MatrixXd PHt = P_ * Ht;	
  MatrixXd K = PHt * Si;	
  //new estimate	
  x_ = x_ + (K * y);	
  long x_size = x_.size();	
  MatrixXd I = MatrixXd::Identity(x_size, x_size);	
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float px = z(0);
  float py = z(1);
  float vx = z(2);
  float vy = z(3);
  float ro = sqrt(px*px + py*py);
  float theta = atan2(py,px);
  float ro_dot = (px*vx + py*vy)/ro;
  VectorXd z_pred = VectorXd(3);
  z_pred << ro, theta, ro_dot;
  VectorXd y = z - z_pred;	
  MatrixXd Ht = H_.transpose();	
  MatrixXd S = H_ * P_ * Ht + R_;	
  MatrixXd Si = S.inverse();	
  MatrixXd PHt = P_ * Ht;	
  MatrixXd K = PHt * Si;	
  //new estimate	
  x_ = x_ + (K * y);	
  long x_size = x_.size();	
  MatrixXd I = MatrixXd::Identity(x_size, x_size);	
  P_ = (I - K * H_) * P_;
  
}
