# BLIMPSLAM
In this work, Drone Localization using [ORBSLAM2](https://github.com/raulmur/ORB_SLAM2) and EKF-Based SensorFusion is investigated. We provide code for dataset collection and integration, visual slam implementation using monocular ORB slam and an EKF sensor fusion class for correction using IMU measurements. 

# HARDWARE REQUIREMENTS

## Data Gathering:

ReadIMU requires an [Arduino UNO](https://store.arduino.cc/usa/arduino-uno-rev3)

record_data requires a [Raspberry PI 3](https://www.raspberrypi.org/products/raspberry-pi-3-model-b/)

# SOFTWARE REQUIREMENTS

## Data Gathering:

ReadIMU requires the following libraries to be installed for Arduino

* Wire
* SPI
* SparkFunLSM9DS1
* SoftwareSerial
* I2C

`record_data.py` requies the following libraries to be installed for the Raspberry PI (use pip3)
`record_data.py` must be run with Python3

* numpy
* cv2
* serial
* time
* csv
* keyboard


## ORB SLAM 2
The ORBSLAM2 library was used for visual SLAM:https://github.com/raulmur/ORB_SLAM2

Files created by author gaoxiang12@github.com

`CMakeLists.txt` is added with 4 lines in the bottom to support running a video

If you want to run a video_ORBSLAM, replace the `CMakeLists.txt` in ORBSLAM with this one or add the 4 lines in it. Move `myvideo.cpp` and `Drone_Cam.yaml` to 
```
ORB_SLAM2/Example/Monocular
```
Change the file path in `myvideo.cpp` with the right one in your folder

```
cd ORB_SLAM2
chmod +x build.sh
./build.sh
```

```
cd ORB_SLAM2/Example/Monocular
./myvideo
```
### Prerequisites for ORB SLAM

* C++11 or C++0x Compiler
* Pangolin: https://github.com/stevenlovegrove/Pangolin.
* OpenCV: http://opencv.org
* Eigen3: http://eigen.tuxfamily.org
* DBoW2 and g2o (Included in Thirdparty folder of ORB)

## EKF Sensor Fusion
The EkfSensorFusion Class is built to merge IMU and ORB_SLAM2 pose information to obtain a position estimate of the drone, in which Extended Kalman Filter (EKF) is used. 

`EkfSensorFusion.h` and `EkfSensorFusion.cpp` include all the class information needed to perfrom sensor fusion of IMU and ORB_SLAM2. Call function `apply_sensorFusion(double time, VectorXf imu_meas, VectorXf cam_meas)` or call functions `prediction()`, `correction_imu(double time, VectorXf imu_meas)`, `correction_cam(VectorXf cam_meas)` seperately to achieve EKF Sensor Fusion.

### Prerequisites for EkfSensorFusion Class

* C++11 or C++0x Compiler
* Eigen3: http://eigen.tuxfamily.org

### Test

`test.cpp` includes `readFile.h` to read the IMU and ORB_SLAM2 data, and matches them  to perform sensor fusion according  to the time stamps. 

If you want to test the class on the data collected from IMU and ORB_SLAM2, use g++ command to compile both `test.cpp` and `EkfSensorFusionClass.cpp` and run `test` in your folder .

```
g++ -I /usr/include/eigen3 test.cpp EkfSensorFusionClass.cpp -o test
./test
```
If you want to change the data being tested, adjust the form of your data as `Interpolated_paused_start_indoor_processed.txt` for IMU data and `KeyFrameTrajectory_paused.txt` for ORB_SLAM2 data. Then move the data file of `.txt` to the folder. For `test.cpp` file, change `filename` and `num1`, `num2`, the number of rows correspondingly.

# Contributing
This is the  project of Team 13, supported by the class ROB530/ EECS568/ NA568: Mobile Robotics, Winter 2020 in University of Michigan. 
Duncan Abbot has contributed to drone configuration, camera calibration, video and data collection. Ahmed Alkatheeri and Hao Weng has implemented and debugged the amazing sensor fusion code. Annet George and Chengfeng Xu has done research and studying on ORB SLAM system configuration, data processing, performance analysis and trajectory plotting.
