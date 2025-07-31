#include <unordered_map>
#include <vector>
#include <string>

std::unordered_map<int, int> urdf_to_sdk_Index = {
    // Right leg
    {5, 8},     //right_hip_yaw
    {6, 0},     //right_hip_roll
    {7, 1},     //right_hip_pitch
    {8, 2},     //right_knee
    {9, 11},         //right_ankle
    {0, 7},      //left_hip_yaw
    {1, 3},      //left_hip_roll
    {2, 4},      //left_hip_pitch
    {3, 5},      //left_knee
    {4, 10},     //left_ankle
    {10, 6},       //waist_yaw
    {15, 12},      //right_shoulder_pitch
    {16, 13},      //right_shoulder_roll
    {17, 14},      //right_shoulder_yaw
    {18, 15},      //right_elbow
    {11, 16},       //left_shoulder_pitch
    {12, 17},       //left_shoulder_roll
    {13, 18},       //left_shoulder_yaw
    {14, 19}        //left_elbow
};

std::unordered_map<int, int>  sdk_to_urdf_Index = {
    {8, 5},     //right_hip_yaw
    {0, 6},     //right_hip_roll
    {1, 7},     //right_hip_pitch
    {2, 8},     //right_knee
    {11, 9},         //right_ankle
    {7, 0},      //left_hip_yaw
    {3, 1},      //left_hip_roll
    {4, 2},      //left_hip_pitch
    {5, 3},      //left_knee
    {10, 4},     //left_ankle 
    {6, 10},       //waist_yaw
    {9, -1},     //not_used_joint
    {12, 15},      //right_shoulder_pitch
    {13, 16},      //right_shoulder_roll
    {14, 17},      //right_shoulder_yaw
    {15, 18},      //right_elbow
    {16, 11},       //left_shoulder_pitch
    {17, 12},       //left_shoulder_roll
    {18, 13},       //left_shoulder_yaw
    {19, 14}        //left_elbow
};

//gets a vector of the torques as input (size 19 ), where the torques are orderd after the urdf indices
//the function returns the vector with the torques ordered after the sdk indices (size 20)
std::vector<double> change_to_sdk_indices (std::vector<double> urdf_torques){

    std::vector<double> sdk_torques(19, 0.0);

    for(int i = 0; i < urdf_torques.size(); i++){
        if(i < 0 || i > 18){ break;}
        sdk_torques[urdf_to_sdk_Index[i]] = urdf_torques[i];
    }

    sdk_torques[9] = 0.0; // TODO: should be the current torque of the waist joint 


    return sdk_torques;
}

//gets a vector of the torques as input (size 20), where the torques are orderd after the sdk indices
//the function returns the vector with the torques ordered after the urdf indices (size 19 )
std::vector<double> change_to_urdf_indices (std::vector<double> sdk_torques){

    std::vector<double> urdf_torque(20, 0.0);

    for(int i = 0; i < sdk_torques.size(); i++){
        if(i < 0 || i > 19 || sdk_to_urdf_Index[i] < 0){ break;}
        urdf_torque[sdk_to_urdf_Index[i]] = sdk_torques[i];
    }

    return urdf_torque;
}

