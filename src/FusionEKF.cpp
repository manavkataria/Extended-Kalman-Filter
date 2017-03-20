#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

#pragma clang diagnostic push
#pragma ide diagnostic ignored "IncompatibleTypes"
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

// Tracking original repo, from `Merge pull request #11 from alexxucui/test`
// commit sha b2cbf6e622d7b4565893c1d1b0f07de62dab5a08

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;

    previous_timestamp_ = 0;

    ekf_.P_ <<  1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1000, 0,
                0, 0, 0, 1000;

    // TODO(Manav): Q init() Probably not needed. Delme.
    // Initialized before prediction stage using `dt`, `noise_ax`, `noise_ay`

    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);

    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
                0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;


    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;

    // Will be overwritten by tools.CalculateJacobian()
    Hj_ << 1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1, 0;
    /**
    TODO: Set the process noise
      * Finish initializing the FusionEKF.
      * Set the process (pending) and measurement noises (done below)
    */

    // Used in Process Covariance Matrix Q
    noise_ax_ = 9;
    noise_ay_ = 9;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO: Done.
      * Initialize the state ekf_.x_ with the first measurement. (done)
      * Create the covariance matrix. (done in KalmanFilter constructor)
      * Remember: you'll need to convert radar from polar to cartesian coordinates. (done)
    */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);      // Allocated in KalmanFilter constructor
    ekf_.x_ << -1, -1, -1, -1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
        float ro = (float) measurement_pack.raw_measurements_[0];
        float phi = (float) measurement_pack.raw_measurements_[1];
        float rodot = (float) measurement_pack.raw_measurements_[2];
        // TODO(Manav): Optional. Try to incorporate rodot for this init
        ekf_.x_ << ro * cos(phi), ro * sin(phi), 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
        /**
        Initialize state.
        */
        ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /* *
     * TODO: Done.
     * Update the state transition matrix F according to the new elapsed time. (done)
      - Time is measured in seconds. (done)
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix. (done)
   */

  // compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1,  0,
             0, 0, 0,  1;

  ekf_.Q_ << pow(dt, 4)/4*noise_ax_, 0, pow(dt, 3)/2*noise_ax_, 0,
             0, pow(dt, 4)/4*noise_ay_, 0, pow(dt, 3)/2*noise_ay_,
             pow(dt, 3)/2*noise_ax_, 0, pow(dt, 2)*noise_ax_, 0,
             0, pow(dt, 3)/2*noise_ay_, 0, pow(dt, 2)*noise_ay_;

  // updates x_ and P_ with predictions
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO: Done. EKF Update from FusionEKF.
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        VectorXd z = VectorXd(3);
        z <<    measurement_pack.raw_measurements_[0],
                measurement_pack.raw_measurements_[1],
                measurement_pack.raw_measurements_[2];
        ekf_.R_ = R_radar_;
        Hj_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.H_ = Hj_;
        ekf_.UpdateEKF(z);
    } else {
        // Laser updates
        VectorXd z = VectorXd(2);
        z <<    measurement_pack.raw_measurements_[0],
                measurement_pack.raw_measurements_[1];
        ekf_.R_ = R_laser_;
        ekf_.H_ = H_laser_;
        ekf_.Update(z);
    }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}

#pragma clang diagnostic pop