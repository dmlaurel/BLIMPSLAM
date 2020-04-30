# EKF Sensor Fusion

`EkfSensorFusion.h` and `EkfSensorFusion.cpp` include all the class information needed to perfrom sensor fusion of IMU and ORB_SLAM2.

`readFile.h` includes the functions used in `test.cpp` to read the `.txt` data files.

Taking advantage of MATLAB, `get_jacbian_cam.m` and `get_jacobian_imu.m` provide with the Jacobian matrices of the motion models needed in EKF.

All other files provide data to be tested.

# Future Work
Introduce correction steps with height sensing information and GPS measurements to further estimate the pose of a drone.