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
