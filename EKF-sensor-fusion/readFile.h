

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;

bool readFileIMU(string fileName, double imu_meas[][6], int num);

bool readFileCAM(string fileName, double cam_meas[][8], int num);

// int main()
// {
//   double imu_meas[1228][4];
//   string fileName = "data.txt";
//   readFile2D(fileName, imu_meas, 1228);  
//   return 0;
// }

bool readFileIMU(string fileName, double imu_meas[][6], int num)
{
  ifstream inFile;
  istringstream iss;
  string line, tag;
  inFile.open(fileName.c_str());

  if (inFile.fail())
  {
    cout << "[ERROR] Unable to open " << fileName << endl;
    return false;
  }

  for (int i = 0; i < num; i++)
  {
    getline(inFile, line);
    iss.str(line);
    // iss >> tag;
    for (int j = 0; j < 6; j++)
    {
      iss >> imu_meas[i][j];
    }
    iss.clear();
  }

  inFile.close();

  return true;
}

bool readFileCAM(string fileName, double cam_meas[][8], int num)
{
  ifstream inFile;
  istringstream iss;
  string line, tag;
  inFile.open(fileName.c_str());

  if (inFile.fail())
  {
    cout << "[ERROR] Unable to open " << fileName << endl;
    return false;
  }

  for (int i = 0; i < num; i++)
  {
    getline(inFile, line);
    iss.str(line);
    // iss >> tag;
    for (int j = 0; j < 8; j++)
    {
      iss >> cam_meas[i][j];
      //std::cout<<cam_meas[i][j]<<" ";
    }
    //std::cout<<endl;
    iss.clear();
  }


  inFile.close();
  return true;
}

bool read_data1_not_csv(string fileName, double all_meas[][8], int num)
{
  ifstream inFile;
  istringstream iss;
  string line, tag;
  inFile.open(fileName.c_str());

  if (inFile.fail())
  {
    cout << "[ERROR] Unable to open " << fileName << endl;
    return false;
  }

  for (int i = 0; i < num; i++)
  {
    getline(inFile, line);
    iss.str(line);
    // iss >> tag;
    for (int j = 0; j < 14; j++)
    {
      iss >> all_meas[i][j];
      //std::cout<<all_meas[i][j]<<" ";
    }
    //std::cout<<endl;
    iss.clear();
  }


  inFile.close();
  return true;
}

bool read_data2_not_csv(string fileName, double all_meas[][9], int num)
{
  ifstream inFile;
  istringstream iss;
  string line, tag;
  inFile.open(fileName.c_str());

  if (inFile.fail())
  {
    cout << "[ERROR] Unable to open " << fileName << endl;
    return false;
  }

  for (int i = 0; i < num; i++)
  {
    getline(inFile, line);
    iss.str(line);
    // iss >> tag;
    for (int j = 0; j < 14; j++)
    {
      iss >> all_meas[i][j];
      //std::cout<<all_meas[i][j]<<" ";
    }
    //std::cout<<endl;
    iss.clear();
  }


  inFile.close();
  return true;
}

/*
  This function takes in the imu measurements in the order of:
    time, real_time, accelerations x, y, z, angles x, y, z, height
  but in a different axes where z is downwards, x is to the right, y is backwards.
  And converts them to the orbslam axes of z forwards, y downwards, x to the right.
*/
void rearrange_imu_meas(double imu_meas[][9], int num){
  /*
  acceleration in x is the same.
  acceleration in y is negative acceleration in z 
  acceleration in z is acceleration in y 
  angle about x is the same 
  angle about y is negative angle about z 
  angle about z is angle about y 
  */
 double temp_meas[9];
 int i, j;
 for(int i = 0; i < num; i++){
    for(j = 0; j<9; j++){
      temp_meas[j] = imu_meas[i][j];
    }

    imu_meas[i][3] = -temp_meas[4]; // acceleration in y is negative acceleration in z 
    imu_meas[i][4] = temp_meas[3]; // acceleration in z is acceleration in y 
    imu_meas[i][6] = -temp_meas[7]; // angle about y is negative angle about z 
    imu_meas[i][7] = temp_meas[6]; // angle about z is angle about y 
 }
}



bool write_data(string fileName, double time[], double cam_meas[][6], double corrected_pose[][6], int num)
{
  ofstream outFile;
  outFile.open(fileName.c_str());

  if (outFile.fail())
  {
    cout << "[ERROR] Unable to open " << fileName << endl;
    return false;
  }

  for (int i = 0; i < num; i++)
  {
    outFile << time[i] << " " << cam_meas[i][0] << " " << cam_meas[i][1] << " " << cam_meas[i][2] << " " << cam_meas[i][3] << " " << cam_meas[i][4] << " " << cam_meas[i][5] << " " << corrected_pose[i][0] << " " << corrected_pose[i][1] << " " << corrected_pose[i][2] << " " << corrected_pose[i][3] << " " << corrected_pose[i][4] << " " << corrected_pose[i][5] << endl;
  }


  outFile.close();
  return true;
}



