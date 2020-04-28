/*
File to handle imu data.
Reads imu angles and accelerations and obtains the change in motion from them. 
*/

#ifndef imu_reader_H_
#define imu_reader_H_

#include "eigen3/Eigen/Dense"
#include "measurement_package.h"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class imuReader{
public:
  /**
  * Constructor.
  */
  imuReader(){
  	dt = 0;
  	error = 0.1;
  	imu_cov = MatrixXd::Identity(6,6) * error;
  	prev_measurements = VectorXd::Zero(9);
  	current_measurements = VectorXd::Zero(9);
  	position_change = VectorXd::Zero(3);
  	velocity_change = VectorXd::Zero(3);
  	prev_velocity = VectorXd::Zero(3);
  	prev_time = 0;
  }

  void update_imu(VectorXd(9) measurements, double time)
  {
  	prev_measurements = current_measurements;
  	current_measurements = measurements;
  	integrate(time);
  }

  void integrate(double new_time)
  {
  	// get dt and update previous time
  	dt = new_time-prev_time; 
  	prev_time = new_time;

  	velocity_change(1) = (current_measurements(4) - prev_measurements(4))*dt; // dvx
  	velocity_change(2) = (current_measurements(5) - prev_measurements(5))*dt; // dvy
  	velocity_change(3) = (current_measurements(6) - prev_measurements(6))*dt; // dvz

  	position_change(1) = velocity_change(1)



  }

private:
	// Measurements will be: gyro x y z, acceleration x y z, pitch roll yaw
	VectorXd prev_measurements;// = VectorXd(9);
	VectorXd current_measurements;// = VectorXd(9);

	// dx dy dz based on integration of accelerations
	VectorXd position_change;// = VectorXd(3);

	// dVx dVy dVz based on integration of acceleration
	VectorXd velocity_change;
	VectorXd prev_velocity;

	// difference in time between previous call and new one
	// Used for integration of acceleration
	double dt;
	double prev_time;

	// covariance error value. Temporary
	double error;// = 0.1;

	// Covariance matrix of IMU measurements:
	// Corresponds to x y z, pitch roll yaw
	MatrixXd imu_cov;// = MatrixXd::Identity(6,6) * error;

};
#endif

  EKFEstimator()
  : Cov_(Eigen::Matrix<double, 7, 7>::Identity() * 1.0),
    Q_(Eigen::Matrix3d::Identity() * 0.01),
    R_(Eigen::Matrix3d::Identity() * 0.033),
    gravity_vec_(Eigen::Vector3d(0.0, 0.0, 9.8067)),
    tau_gyro_bias_{1.0}
  {
    /* x =[qw qx qy qz gyro_bias_x gyro_bias_y gyro_bias_z] */
    x_ << 1, 0, 0, 0, 0, 0, 0;
  }
  void setdt(const double dt)
  {
    dt_ = dt;
  }

  void setProcessNoize(const double process_noize)
  {
    Q_ = Eigen::Matrix3d::Identity() * process_noize;
  }


  void setObservationNoize(const double observation_noize)
  {
    R_ = Eigen::Matrix3d::Identity() * observation_noize;
  }

private:
  Eigen::Matrix<double, 7, 1> x_;
  Eigen::Matrix<double, 7, 7> Cov_;
  double dt_;

  Eigen::Matrix3d Q_, R_;
  Eigen::Vector3d gravity_vec_;
  double tau_gyro_bias_;
};

#endif  // IMU_ESTIMATOR__EKF_HPP_
