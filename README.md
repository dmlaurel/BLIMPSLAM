# BLIMPSLAM

SOFTWARE REQUIREMENTS

Data Gathering:

ReadIMU requires the following libraries to be installed for Arduino

-Wire
-SPI
-SparkFunLSM9DS1
-SoftwareSerial
-I2C

record_data.py requies the following libraries to be installed for the Raspberry PI (use pip3)
record_data.py must be run with Python3

-numpy
-cv2
-serial
-time
-csv
-keyboard

HARDWARE REQUIREMENTS

Data Gathering:

ReadIMU requires an Arduino UNO (LINK)

record_data requires a Raspberry PI 3 (LINK)

Third Party Repositories: 
ORB SLAM: https://github.com/raulmur/ORB_SLAM2



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


