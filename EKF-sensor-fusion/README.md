# EKF Sensor Fusion

To compile, run the following command in the terminal
```
g++ -I /usr/include/eigen3 test.cpp EkfSensorFusionClass.cpp -o test
```

To install Eigen3, run the following command in the terminal
```
sudo apt install libeigen3-dev
```

`EkfSensorFusionClass.h` and `EkfSensorFusionClass.cpp` include all the class information needed to perfrom sensor fusion of IMU and ORB_SLAM2.

`test.cpp` tests the class.

`readFile.h` includes the functions used in `test.cpp` to read the `.txt` data files.

Taking advantage of MATLAB, `get_jacbian_cam.m` and `get_jacobian_imu.m` provide with the result of Jacobian matrices corresponding to the motion models needed in EKF. 

All other files provide data to be tested.

# Future Work
Introduce correction steps with height sensing information and GPS measurements to further correct the pose of the drone.
