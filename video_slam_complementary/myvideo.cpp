// Autohor gaoxiang12@github.com
// Created by xiang on 11/29/17.
//

// open video file，pass images to ORB-SLAM2 to relocalize

// include opencv
#include <opencv2/opencv.hpp>

// ORB-SLAM interface
#include "System.h"

#include <string>
#include <chrono>   // for time stamp
#include <iostream>

using namespace std;

// parameter and dictionary files
// change to right path on your computer
string parameterFile = "PATH TO Drone_Cam.yaml";
string vocFile = "/home/usr_name/ORB_SLAM2/Vocabulary/ORBvoc.txt";

// video file
string videoFile = "PATH TO video";

int main(int argc, char **argv) {

    // declare ORB-SLAM 
    ORB_SLAM2::System SLAM(vocFile, parameterFile, ORB_SLAM2::System::MONOCULAR, true);

    // images from video
    cv::VideoCapture cap(videoFile);    // change to 1 if you want to use USB camera.

    // system time
    auto start = chrono::system_clock::now();

    while (1) {
        cv::Mat frame;
        cap >> frame;   // camera data
        if ( frame.data == nullptr )
            break;

        // rescale because image is too large
        cv::Mat frame_resized;
        cv::resize(frame, frame_resized, cv::Size(640,360));

        auto now = chrono::system_clock::now();
        auto timestamp = chrono::duration_cast<chrono::milliseconds>(now - start);
        SLAM.TrackMonocular(frame_resized, double(timestamp.count())/1000.0);
        cv::waitKey(30);
    }

    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    return 0;
}
