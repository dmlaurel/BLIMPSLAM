/*
C++ Sensor fusion code 
Overall content:
1- Define class
2- Define variables accessible from outside
3- Define EKF functions
4- Define motion model with random gaussian noise sampled

To compile:
g++ -I /usr/include/eigen3 test.cpp EkfSensorFusionClass.cpp -o test

To debug: print using:
std::cout << "Message 1 "<< endl;

*/

#include "EkfSensorFusionClass.h"


ekfSensorFusion::ekfSensorFusion(){
	ekfSensorFusion::initialize();
}


void ekfSensorFusion::initialize(){
	// initialize pose
	mu_pose_pred = VectorXf::Zero(6);
	sigma_pose_pred = MatrixXf::Zero(6,6);
	mu_pose = VectorXf::Zero(6);
	sigma_pose = MatrixXf::Zero(6,6);

	// initialize scale
	scale = 8.5; // scale corresponding to 4 meter forward drive. //1.0;

	// initialize random walk noise variance
	noise_var = 0.01;

	// initialize random noise generator
	distribution = normal_distribution<double> (0,sqrt(noise_var)); // 0 mean noise

	// initialize Prediction Jacobians:
	G = MatrixXf::Identity(6,6); // Identity Jacobian since it is a random walk model
	V = MatrixXf::Identity(6,6); // Identity Jacobian since it is a random walk model

	// initializing sensor covariance matrices
	double imu_angle_var = 0.01;
	R_IMU = MatrixXf::Identity(3,3) * imu_angle_var;

	R_CAM = MatrixXf::Zero(7,7);
	double cam_position_var = 0.01;
	double cam_angle_var = 0.1;
	R_CAM.block(0,0,3,3) = cam_position_var * MatrixXf::Identity(3,3); // first secion is position covariance
	R_CAM.block(3,3,4,4) = cam_angle_var * MatrixXf::Identity(4,4); // second secion is angle covariance
	
	R_height = 0;
	
}

void ekfSensorFusion::prediction()
{
	// Create random noise vector and covariance
	MatrixXf noise_cov = MatrixXf::Identity(6,6) * noise_var;
	VectorXf noise = VectorXf::Zero(6);
	for(int i = 0; i<6; i++) // go through position and angles and create noise
	{
		noise(i) = distribution(generator);
	}

	// Get predicted pose and sigma
	mu_pose_pred = mu_pose + noise;
	sigma_pose_pred = G*sigma_pose*G.transpose() + V*noise_cov*V.transpose(); 
}

MatrixXf ekfSensorFusion::get_imu_jacobian()
{
	MatrixXf H = MatrixXf::Identity(3,3);

	return H;
}

void ekfSensorFusion::correction_imu(double time, VectorXf imu_meas)
{	
	
	// Jacobian based on equations of observation model
	MatrixXf H = get_imu_jacobian(); // takes in the acceleration before the rotation

	VectorXf z_hat = VectorXf(3);
	z_hat = mu_pose_pred.tail(3);

	// get Kalman gain
	MatrixXf S = R_IMU + H*sigma_pose_pred.block(3,3,3,3)*H.transpose();
	MatrixXf K = sigma_pose_pred.block(3,3,3,3) * H.transpose() * S.inverse();

	// get new mean and covariance
	// Update angles but keep position coordinates as they are
	mu_pose.head(3) = mu_pose_pred.head(3);
	mu_pose.tail(3) = mu_pose_pred.tail(3) + K*(imu_meas.tail(3) - z_hat);
	sigma_pose.block(0,0,3,3) = sigma_pose_pred.block(0,0,3,3);
	sigma_pose.block(3,3,3,3) = (MatrixXf::Identity(3,3) - K*H)*sigma_pose_pred.block(3,3,3,3);
}


void ekfSensorFusion::correction_cam(VectorXf cam_meas)
{	
	// scale camera measurements to real world measurements
	cam_meas = scale_cam_meas(cam_meas);

	VectorXf z_hat = VectorXf(7);		
	z_hat.head(3) = mu_pose.head(3);
	z_hat.tail(4) = angle_to_quat(mu_pose.tail(3));

	// Jacobian based on equations of observation model
	MatrixXf J_c = get_cam_jacobian();					

	// get Kalman gain
	MatrixXf S = R_CAM + J_c*sigma_pose*J_c.transpose();
	MatrixXf K = sigma_pose * J_c.transpose() * S.inverse();
	
	// get new mean and covariance
	mu_pose = mu_pose + K*(cam_meas - z_hat);
	sigma_pose = (MatrixXf::Identity(6,6) - K*J_c)*sigma_pose;
}	


MatrixXf ekfSensorFusion::get_cam_jacobian(){
	VectorXf angle = mu_pose.tail(3);
	MatrixXf J = MatrixXf::Zero(7,6);
	// found the equtions using matlab
	
	J.block(0,0,3,3) = MatrixXf::Identity(3,3);
	
	J(3, 3) = (cos(angle(0)/2)*cos(angle(1)/2)*cos(angle(2)/2))/2 + (sin(angle(0)/2)*sin(angle(1)/2)*sin(angle(2)/2))/2;
	J(3, 4) = - (cos(angle(0)/2)*cos(angle(1)/2)*sin(angle(2)/2))/2 - (cos(angle(2)/2)*sin(angle(0)/2)*sin(angle(1)/2))/2;
	J(3, 5) = - (cos(angle(0)/2)*cos(angle(2)/2)*sin(angle(1)/2))/2 - (cos(angle(1)/2)*sin(angle(0)/2)*sin(angle(2)/2))/2;
	
	J(4, 3) = (cos(angle(0)/2)*cos(angle(1)/2)*sin(angle(2)/2))/2 - (cos(angle(2)/2)*sin(angle(0)/2)*sin(angle(1)/2))/2;
	J(4, 4) =  (cos(angle(0)/2)*cos(angle(1)/2)*cos(angle(2)/2))/2 - (sin(angle(0)/2)*sin(angle(1)/2)*sin(angle(2)/2))/2;
	J(4, 5) =  (cos(angle(0)/2)*cos(angle(1)/2)*cos(angle(2)/2))/2 + (sin(angle(0)/2)*sin(angle(1)/2)*sin(angle(2)/2))/2;
	
	J(5, 3) = - (cos(angle(0)/2)*cos(angle(2)/2)*sin(angle(1)/2))/2 - (cos(angle(1)/2)*sin(angle(0)/2)*sin(angle(2)/2))/2;			
	J(5, 4) = - (cos(angle(1)/2)*cos(angle(2)/2)*sin(angle(0)/2))/2 - (cos(angle(0)/2)*sin(angle(1)/2)*sin(angle(2)/2))/2;
	J(5, 5) = (cos(angle(0)/2)*cos(angle(1)/2)*cos(angle(2)/2))/2 + (sin(angle(0)/2)*sin(angle(1)/2)*sin(angle(2)/2))/2;
	
	J(6,3) = (cos(angle(0)/2)*sin(angle(1)/2)*sin(angle(2)/2))/2 - (cos(angle(1)/2)*cos(angle(2)/2)*sin(angle(0)/2))/2;
	J(6,4) = (cos(angle(1)/2)*sin(angle(0)/2)*sin(angle(2)/2))/2 - (cos(angle(0)/2)*cos(angle(2)/2)*sin(angle(1)/2))/2;
	J(6,5) = (cos(angle(2)/2)*sin(angle(0)/2)*sin(angle(1)/2))/2 - (cos(angle(0)/2)*cos(angle(1)/2)*sin(angle(2)/2))/2;

	return J;

}

void ekfSensorFusion::get_scale(VectorXf cam_meas1, VectorXf cam_meas2, double height1, double height2){
	double z_diff_cam = cam_meas2(1) - cam_meas1(1); // get difference in the y-axis (corresponding to height)
	double z_diff = height2 - height1;
	scale = -z_diff / z_diff_cam;

	scale /= 1000.0; // converts scale to meters instead of mm
}

VectorXf ekfSensorFusion::scale_cam_meas(VectorXf cam_meas){
	cam_meas.head(3) *= scale;
	return cam_meas;
}

VectorXf ekfSensorFusion::apply_sensorFusion(double time, VectorXf imu_meas, VectorXf cam_meas){
	prediction();
	std::cout << "pose after prediction\n "<< mu_pose_pred << endl;
	correction_imu(time, imu_meas);
	std::cout << "pose after imu correction\n "<< mu_pose << endl;
	correction_cam(cam_meas);	
	std::cout << "pose after cam correction\n "<< mu_pose << endl;
	return mu_pose;	
}

VectorXf ekfSensorFusion::get_mu_pose(){
	return mu_pose;
}

VectorXf ekfSensorFusion::get_mu_pose_pred(){
	return mu_pose_pred;
}

VectorXf ekfSensorFusion::angle_to_quat(VectorXf angles){
	// yaw (Z) (2), pitch (Y) (1), roll (X) (0) now
	//pitch (Y) (0), roll (X) (1), yaw (Z) (2)
	// Abbreviations for the various angular functions
	angles = angles*M_PI/180;

	double cy = cos(angles(2) * 0.5);
	double sy = sin(angles(2) * 0.5);
	double cp = cos(angles(1) * 0.5);
	double sp = sin(angles(1) * 0.5);
	double cr = cos(angles(0) * 0.5);
	double sr = sin(angles(0) * 0.5);

	VectorXf q = VectorXf(4);
	double x = sr * cp * cy - cr * sp * sy;
	double y = cr * sp * cy + sr * cp * sy;
	double z = cr * cp * sy - sr * sp * cy;
	double w = cr * cp * cy + sr * sp * sy;

	q << x, y, z, w;

	return q;
}

VectorXf ekfSensorFusion::quat_to_angle(VectorXf q){
	// roll (x-axis rotation)
	double sinr_cosp = 2 * (q(3) * q(0) + q(1) * q(2));
	double cosr_cosp = 1 - 2 * (q(0) * q(0) + q(1) * q(1));
	double roll = std::atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = 2 * (q(3) * q(1) - q(2) * q(0));
	double pitch;
	if (std::abs(sinp) >= 1)
		pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = std::asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = 2 * (q(3) * q(2) + q(0) * q(1));
	double cosy_cosp = 1 - 2 * (q(1) * q(1) + q(2) * q(2));
	double yaw = std::atan2(siny_cosp, cosy_cosp);

	roll = roll*180/M_PI;
	pitch = pitch*180/M_PI;
	yaw = yaw*180/M_PI;
	
	VectorXf angles = VectorXf(3);
	angles << roll, pitch, yaw;

	return angles;
}
