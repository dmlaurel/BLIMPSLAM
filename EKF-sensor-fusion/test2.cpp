/*
To compile:
g++ -I /usr/include/eigen3 test2.cpp EkfSensorFusionClass.cpp -o test
*/
#include "readFile.h"
#include "EkfSensorFusionClass.h"


int main()
{
// This is the test with the paused video data:
    // Read imu data. From interpolated paused start indoor file.
    string fileName = "Interpolated_paused_start_indoor_processed2.txt";
    int num1 = 32;
    double all_meas[num1][9];
    VectorXf imu_meas = VectorXf(6);
    read_data2_not_csv(fileName, all_meas, num1);
    rearrange_imu_meas(all_meas, num1); // rearrange the imu measurements to match orbslam axes

    // Read cam data. From key frame trajectory paused.
    fileName = "KeyFrameTrajectory_paused.txt";
    int num2 = 32;
    double all_cam_meas[num2][8];    
    VectorXf cam_meas = VectorXf(7);
    readFileCAM(fileName, all_cam_meas, num2); 

    // variables for output file
    double recorded_cam_meas[num2][6]; 
    double recorded_poses[num2][6];
    double recorded_time[num2];
    fileName = "Recorded poses2.txt";

    double time = 0;
    double cam_time = 0;
    ekfSensorFusion ekf;
    VectorXf pose = VectorXf(6);

/*
    // obtain scale:
    VectorXf cam_meas1 = VectorXf(7);
    VectorXf cam_meas2 = VectorXf(7);
    double height1, height2;
    height1 = all_meas[0][8];
    height2 = all_meas[num2-1][8];
    cam_meas1 << all_cam_meas[0][1], all_cam_meas[0][2], all_cam_meas[0][3], all_cam_meas[0][4], all_cam_meas[0][5], all_cam_meas[0][6],all_cam_meas[0][7];
    cam_meas2 << all_cam_meas[num2-1][1], all_cam_meas[num2-1][2], all_cam_meas[num2-1][3], all_cam_meas[num2-1][4], all_cam_meas[num2-1][5], all_cam_meas[num2-1][6],all_cam_meas[num2-1][7];
    ekf.get_scale(cam_meas1, cam_meas2, height1, height2);
*/

    int loop_limit = num2; // num1;
    for(int i = 0; i<loop_limit; i++){

        time = all_meas[i][1]/1000; // get time in seconds for the imu math
        cam_time = all_cam_meas[i][0];
        // take in accelerations x, y, z, and angles roll pitch yaw (x, y, z)
        imu_meas << all_meas[i][2], all_meas[i][3], all_meas[i][4], all_meas[i][6], all_meas[i][5], all_meas[i][7];
        cam_meas << all_cam_meas[i][1], all_cam_meas[i][2], all_cam_meas[i][3], all_cam_meas[i][4], all_cam_meas[i][5], all_cam_meas[i][6],all_cam_meas[i][7];
        

        // run prediction step
        ekf.prediction();

        // run imu correction
        ekf.correction_imu(time, imu_meas);
        if(i >= 11)
        {
            pose = ekf.get_mu_pose();
            std::cout << "index is: " << i << " and imu pose is: \n" << pose << endl;
            std::cout << "index is: " << i << " and cam meas is: \n" << cam_meas << endl;

        }

        ekf.correction_cam(cam_meas);

        pose = ekf.get_mu_pose();
        std::cout << "index is: " << i << " and pose is: \n" << pose << endl;

        VectorXf cam_angles = ekf.quat_to_angle(cam_meas.tail(4));
        recorded_time[i] = cam_time;
        for(int j = 0; j<3; j++){
            recorded_cam_meas[i][j] = cam_meas(j);
            recorded_cam_meas[i][j+3] = cam_angles(j);

            recorded_poses[i][j] = pose(j);
            recorded_poses[i][j+3] = pose(j+3);
        }

        if(isnan(pose(0))) break;
        
        
    }
    write_data(fileName, recorded_time, recorded_cam_meas, recorded_poses, num2);
    return 0;
}