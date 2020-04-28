#ifndef EkfSensorFusionClass_H_
#define EkfSensorFusionClass_H_

#include "eigen3/Eigen/Dense"
using namespace std;
using Eigen::MatrixXf;
using Eigen::VectorXf;
// #include <vector>
// #include <string>
// #include <iostream>
#include <math.h>
#include <random>
// #include "quat_lib.h"
#include <iostream>

class ekfSensorFusion{
public:
  /**
  * Constructor.
  */
  ekfSensorFusion();

  // initialize all the necessary variables.
  void initialize();

  /*
    runs the prediction model of the system.
    Adds 0-mean gaussian noise with the initialized noise variance value
    The noise is the same for all the states in the pose
    Random walk model used with corresponding equations and Jacobians.
  */
  void prediction();

  /*
    Function that returns mu_pose
  */
  VectorXf get_mu_pose();

  /*
    Function that returns mu_pose_pred
  */
  VectorXf get_mu_pose_pred();


  /*
    Calculates the Jacobian of the observation model
  */
  MatrixXf get_imu_jacobian();


  /*
    runs the correction using the imu measurements.
    receives time and imu_measurements. 
    imu_measurements: acceleration x, y, z, angles about x, y, z
  */
  void correction_imu(double time, VectorXf imu_meas);

  /*
    Code found on wikipedia: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    Converts angles in degrees to a quaternion vector
    angles expected as roll (X), pitch (Y), yaw (Z)
  */
  VectorXf angle_to_quat(VectorXf angle);

  /*
    Code found on wikipedia: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    Converts quaternion to an angles vector in degrees
    quaternion expected as x, y, z, w
    returns angles as roll (X), pitch (Y), yaw (Z)
  */
  VectorXf quat_to_angle(VectorXf q);

  /*
    runs the correction step using the ORBSLAM2 output pose
    It does it with quaternions, not angles
    Data expected in keyframeresult: timestamp x y z qx qy qz qw
  */
  void correction_cam(VectorXf cam_meas);

  /*
    Calculates the Jacobian of the observation model using the accelerations from get_acceleration
    and the change in time. The equations were obtained using Matlab
  */
  MatrixXf get_cam_jacobian();

  /*
    Function that gets the scale to be used to make the ORBSLAM2 poses scaled to real life.
    It expects 2 orbslam2 poses taken with a change in the height, preferrably as the drone starts flying.
    It takes in 2 height sensor readings at the same time as the camera poses are generated.
    It updates the scale variable in the class.
  */
   void get_scale(VectorXf cam_meas1, VectorXf cam_meas2, double height1, double height2);

  /*
    This function scales the orbslam2 measurements to real life measurements.
  */
   VectorXf scale_cam_meas(VectorXf cam_meas);

  /*
    Function that calls the prediction step then correction steps using imu and camera.
    It returns the corrected pose from sensor fusion.
    It expects imu_meas to contain: accelerations x, y, z, angles x, y, z (roll pitch yaw)
    It expects cam_meas to contain: x, y, z, quaternions qx, qy, qz, qw
  */
  VectorXf apply_sensorFusion(double time, VectorXf imu_meas, VectorXf cam_meas);



  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  //void ProcessMeasurement(const MeasurementPackage &measurement_pack);


private:
  // pose is: x, y, z, angle_x, angle_y, angle_z
  VectorXf mu_pose_pred;
  MatrixXf sigma_pose_pred;
  VectorXf mu_pose;
  MatrixXf sigma_pose;

  // scale for camera measurements from ORBSLAM2
  double scale;

  // sensor covariances:
  MatrixXf R_IMU;
  MatrixXf R_CAM;
  float R_height;

  // motion model Jacobians:
  MatrixXf G; // Motion state Jacobian
  MatrixXf V; // input Jacobian

  // random noise generator
  default_random_engine generator;
  double noise_var;
  normal_distribution<double> distribution;
};

#endif
