#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

#define SmallValue 0.0001

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

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

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
	  

	  
	  //measurement matrix
	  H_laser_ << 1, 0, 0, 0,
				0, 1, 0, 0;

	  

	  
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
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 0, 0, 0, 0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      ekf_.x_(0) = measurement_pack.raw_measurements_(0)*cos(measurement_pack.raw_measurements_(1));
	  ekf_.x_(1) = measurement_pack.raw_measurements_(0)*sin(measurement_pack.raw_measurements_(1));
	  ekf_.x_(2) = measurement_pack.raw_measurements_(2)*cos(measurement_pack.raw_measurements_(1));
	  ekf_.x_(3) = measurement_pack.raw_measurements_(2)*sin(measurement_pack.raw_measurements_(1));
	  
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_(0) = measurement_pack.raw_measurements_(0);
	  ekf_.x_(1) = measurement_pack.raw_measurements_(1);

    }

	if(fabs(ekf_.x_(0)) < SmallValue and fabs(ekf_.x_(1)) < SmallValue)
	{
		ekf_.x_(0) = SmallValue;
		ekf_.x_(1) = SmallValue;
	}

	cout << "initial value" << ekf_.x_ <<endl;
		  //state covariance matrix P
	ekf_.P_ = MatrixXd(4, 4);
	ekf_.P_ << 1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, 1000, 0,
				0, 0, 0, 1000;

	previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;

    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
	  //set the acceleration noise components
  float noise_ax = 9.0;
  float noise_ay = 9.0;

  float dt1 = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000;

  previous_timestamp_ = measurement_pack.timestamp_;

  //the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, dt1, 0,
			0, 1, 0, dt1,
			0, 0, 1, 0,
			0, 0, 0, 1;
	  
  float dt2 = dt1 * dt1;
  float dt3 = dt2 * dt1;
  float dt4 = dt3 * dt1;


  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt4*noise_ax/4, 0, dt3*noise_ax/2,0,
  			 0,dt4*noise_ay/4, 0, dt3*noise_ay/2,
  			 dt3*noise_ax/2,0,dt2*noise_ax,0,
  			 0,dt3*noise_ax/2,0,dt2*noise_ax;


  ekf_.Predict();


  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    Hj_ = tools.CalculateJacobian(ekf_.x_);

	ekf_.R_ = R_radar_;
	ekf_.H_ = Hj_;
	
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
	cout << "radar" << measurement_pack.raw_measurements_ << endl;
    
  } else {
    // Laser updates

	ekf_.R_ = R_laser_;
	ekf_.H_ = H_laser_;

    ekf_.Update(measurement_pack.raw_measurements_);
	cout << "lidar" << measurement_pack.raw_measurements_ << endl;
  }


  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
