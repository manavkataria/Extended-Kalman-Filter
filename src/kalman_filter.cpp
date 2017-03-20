#include "kalman_filter.h"
#include<math.h>
#include<iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {
    x_ = VectorXd(4);       // State Vector x
    P_ = MatrixXd(4, 4);    // State Covariance Matrix P
    //H = Allocated in FusionEKF Constructor    // Measurement Matrix H
    //R = Allocated in FusionEKF Constructor    // Measurement Covariance R
    F_ = MatrixXd(4, 4);    // State Transition (aka Process?) Matrix F
    Q_ = MatrixXd(4, 4);    // Process Covariance Matrix Q

}

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
  /*
   * QUESTION(Manav): This works for Laser; What about Radar x_? It does but estimates a Cartesian State.
   * predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO: Done.
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
  TODO: EKF - Done.
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */

    VectorXd h_nonlinear = VectorXd(3);
    h_nonlinear = tools.CalculateHNonLinear(x_);

    // NOTE: Z is in polar 3D coordinate space
    VectorXd y = z - h_nonlinear;                          // NOTE: using h_nonlinear(x) instead of H matrix!
    y(1) = tools.wrapMinMax(y(1), -M_PI, M_PI);            // phi should be in [-pi, pi)

    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    // QUESTION(Manav): Given that y is in polar 3D space should we convert it to scalar before assigning to x?
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}
