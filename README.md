# BLIMPSLAM
In this work, Drone Localization using [ORBSLAM2](https://github.com/raulmur/ORB_SLAM2) and EKF-Based SensorFusion is investigated. We provide code for dataset collection and integration, visual slam implementation using monocular ORB slam and an EKF sensor fusion class for correction using IMU measurements. 

## HARDWARE REQUIREMENTS

Data Gathering:

ReadIMU requires an [Arduino UNO](https://store.arduino.cc/usa/arduino-uno-rev3)

record_data requires a [Raspberry PI 3](https://www.raspberrypi.org/products/raspberry-pi-3-model-b/)

## SOFTWARE REQUIREMENTS
Data Gathering:

ReadIMU requires the following libraries to be installed for Arduino

* Wire
* SPI
* SparkFunLSM9DS1
* SoftwareSerial
* I2C

record_data.py requies the following libraries to be installed for the Raspberry PI (use pip3)
record_data.py must be run with Python3

* numpy
* cv2
* serial
* time
* csv
* keyboard


## ORB SLAM 2
The ORBSLAM2 library was used for visual SLAM:https://github.com/raulmur/ORB_SLAM2

Files created by author gaoxiang12@github.com

CMakeLists.txt is added with 4 lines in the bottom to support running a video

If you want to run a video_ORBSLAM, replace the CMakeLists.txt in ORBSLAM with this one or add the 4 lines in it. Move myvideo.cpp and Drone_Cam.yaml to 
```
ORB_SLAM2/Example/Monocular
```
Change the file path in myvideo.cpp with the right one in your folder

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

