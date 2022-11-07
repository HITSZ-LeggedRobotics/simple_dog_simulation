#include <ros/ros.h>
//#include "servocontrol.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"
#include "string.h"

#include "fstream"
using namespace std;

//std_msgs::Float64 joint1,joint2,joint3,joint4,joint5,joint6,joint7,joint8,joint9,joint10,joint11,joint12;
//int startFlag =0;
std_msgs::Float64MultiArray joint_group_positions;
void jointCommandCallback(const sensor_msgs::JointStateConstPtr& jointCommandMsg)
{
//    joint1.data = jointCommandMsg->position[0];
//    joint2.data = jointCommandMsg->position[1];
//    joint3.data = jointCommandMsg->position[2];
//    joint4.data = jointCommandMsg->position[3];
//    joint5.data = jointCommandMsg->position[4];
//    joint6.data = jointCommandMsg->position[5];
//    joint7.data = jointCommandMsg->position[6];
//    joint8.data = jointCommandMsg->position[7];
//    joint9.data = jointCommandMsg->position[8];
//    joint10.data = jointCommandMsg->position[9];
//    joint11.data = jointCommandMsg->position[10];
//    joint12.data = jointCommandMsg->position[11];
  joint_group_positions.data = jointCommandMsg->position;
//    startFlag = 1;
//    ROS_INFO("Recieved a New Joint Command j1 = %f j2 = %f j3 = %f j4 = %f j5 = %f j6 = %f j7 = %f ",
//             joint1.data,joint2.data,joint3.data,joint4.data,joint5.data,joint6.data,joint7.data);
}
//void resetCommandCallback(const std_msgs::BoolConstPtr& resetCommandMsg)
//{
//    ROS_INFO("got a  Reset");
//    if(resetCommandMsg->data)
//        resetFlag =1;
//}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_command_interface");
    ros::NodeHandle nh;
    joint_group_positions.data.resize(19);
     joint_group_positions.data[0] = 0;
     joint_group_positions.data[1] = 1.4;
     joint_group_positions.data[2] = -2.75;
     joint_group_positions.data[3] = 0;
     joint_group_positions.data[4] = -1.4;
     joint_group_positions.data[5] = 2.75;
     joint_group_positions.data[6] = 0;
     joint_group_positions.data[7] = 1.4;
     joint_group_positions.data[8] = -2.75;
     joint_group_positions.data[9] = 0;
     joint_group_positions.data[10] = -1.4;
     joint_group_positions.data[11] = 2.75;

//     joint_group_positions.data[0] = 0;
//     joint_group_positions.data[1] = 0.7;
//     joint_group_positions.data[2] = -1.4;
//     joint_group_positions.data[3] = 0;
//     joint_group_positions.data[4] = -0.7;
//     joint_group_positions.data[5] = 1.4;
//     joint_group_positions.data[6] = 0;
//     joint_group_positions.data[7] = 0.7;
//     joint_group_positions.data[8] = -1.4;
//     joint_group_positions.data[9] = 0;
//     joint_group_positions.data[10] = -0.7;
//     joint_group_positions.data[11] = 1.4;

//   joint_group_positions.data.resize(19);
//   joint_group_positions.data[0] = 0;
//   joint_group_positions.data[1] = 0.04;
//   joint_group_positions.data[2] = -0.08;
//   joint_group_positions.data[3] = 0;
//   joint_group_positions.data[4] = -0;
//   joint_group_positions.data[5] = 0;
//   joint_group_positions.data[6] = 0;
//   joint_group_positions.data[7] = -0;
//   joint_group_positions.data[8] = 0;
//   joint_group_positions.data[9] = 0;
//   joint_group_positions.data[10] = 0;
//   joint_group_positions.data[11] = 0;


//    joint_group_positions.data[12] = -0;
//    joint_group_positions.data[13] = -1.5 + M_PI/2;
//    joint_group_positions.data[14] = 0;
//    joint_group_positions.data[15] = 2.29 - M_PI /2 ;
//    joint_group_positions.data[16] = 0;
//    joint_group_positions.data[17] = 0.77;
//    joint_group_positions.data[18] = -0;



    std::vector<double> joint_angles_temp_;
    joint_angles_temp_.resize(18);

    std::ifstream readfile;
    readfile.open("/home/kun/catkin_ws_dependency/haibinangles.txt");//calculated.txt
    assert(readfile.is_open());
    readfile >> joint_angles_temp_[0] >> joint_angles_temp_[1]>> joint_angles_temp_[2] >> joint_angles_temp_[3]>> joint_angles_temp_[4]
        >> joint_angles_temp_[5]>> joint_angles_temp_[6]      >> joint_angles_temp_[7] >> joint_angles_temp_[8]>> joint_angles_temp_[9]
        >> joint_angles_temp_[10] >> joint_angles_temp_[11] >> joint_angles_temp_[12] >> joint_angles_temp_[13] >> joint_angles_temp_[14]
        >> joint_angles_temp_[15] >> joint_angles_temp_[16] >> joint_angles_temp_[17];
    std::cout << """""""""""""""""""""""""""""""""""""""""""""" << std::endl;
    std::cout <<"load the joint angle" << std::endl;
    std::cout <<"the joint angles is " << std::endl;

    for (unsigned int i = 0; i < joint_angles_temp_.size(); i++) {
        std::cout << joint_angles_temp_[i] << " ";
    }

    readfile.close();

//    joint_group_positions.data[0] = joint_angles_temp_[0];
//    joint_group_positions.data[1] = joint_angles_temp_[1];
//    joint_group_positions.data[2] = joint_angles_temp_[2];

//    joint_group_positions.data[3] = joint_angles_temp_[3];
//    joint_group_positions.data[4] = joint_angles_temp_[4];
//    joint_group_positions.data[5] = joint_angles_temp_[5];

//    joint_group_positions.data[6] = joint_angles_temp_[6];
//    joint_group_positions.data[7] = joint_angles_temp_[7];
//    joint_group_positions.data[8] = joint_angles_temp_[8];

//    joint_group_positions.data[9] = joint_angles_temp_[9];
//    joint_group_positions.data[10] = joint_angles_temp_[10];
//    joint_group_positions.data[11] = joint_angles_temp_[11];

//    joint_group_positions.data[12] = joint_angles_temp_[12];;
//    joint_group_positions.data[13] = joint_angles_temp_[13];;
//    joint_group_positions.data[14] = 0;
//    joint_group_positions.data[15] = joint_angles_temp_[14];;
//    joint_group_positions.data[16] = joint_angles_temp_[15];;
//    joint_group_positions.data[17] = joint_angles_temp_[16];;
//    joint_group_positions.data[18] = joint_angles_temp_[17];;


    joint_group_positions.data[12] = joint_angles_temp_[12];
    joint_group_positions.data[13] = joint_angles_temp_[13];
    joint_group_positions.data[14] = 0;
    joint_group_positions.data[15] = joint_angles_temp_[14];
    joint_group_positions.data[16] = joint_angles_temp_[15];
    joint_group_positions.data[17] = joint_angles_temp_[16];
    joint_group_positions.data[18] = joint_angles_temp_[17];

//    joint_group_positions.data[0] = 0.317;
//    joint_group_positions.data[1] = 0.363;
//    joint_group_positions.data[2] = -2.67;

//    joint_group_positions.data[3] = 0.1098;
//    joint_group_positions.data[4] = -0.1935;
//    joint_group_positions.data[5] = 1.194;

//    joint_group_positions.data[6] = -0.3098;
//    joint_group_positions.data[7] = 1;
//    joint_group_positions.data[8] = -1.194;

//    joint_group_positions.data[9] = -0.116;
//    joint_group_positions.data[10] = -1.17;
//    joint_group_positions.data[11] = 1.44;


//    joint_group_positions.data[12] = 0.68;
//    joint_group_positions.data[13] = 1.15;
//    joint_group_positions.data[14] = 0;
//    joint_group_positions.data[15] = 1.07;
//    joint_group_positions.data[16] = 0;
//    joint_group_positions.data[17] = 0.778;
//    joint_group_positions.data[18] = 1.53;

    ros::Subscriber jointCommandSub = nh.subscribe("/action_server_test_node/all_joint_position", 1, jointCommandCallback);
    ros::Publisher joint_group_position_pub = nh.advertise<std_msgs::Float64MultiArray>("/all_joints_position_effort_group_controller/command",1);
    //    ros::Subscriber resetCommandSub = nh.subscribe("/reset_flag",1,resetCommandCallback);
//    ros::Publisher joint1ControllerCommandPub =
//            nh.advertise<std_msgs::Float64>("front_left_1_joint_position_controller/command",1);
//    ros::Publisher joint2ControllerCommandPub =
//            nh.advertise<std_msgs::Float64>("front_left_2_joint_position_controller/command",1);
//    ros::Publisher joint3ControllerCommandPub =
//            nh.advertise<std_msgs::Float64>("front_left_3_joint_position_controller/command",1);
//    ros::Publisher joint4ControllerCommandPub =
//            nh.advertise<std_msgs::Float64>("front_right_1_joint_position_controller/command",1);
//    ros::Publisher joint5ControllerCommandPub =
//            nh.advertise<std_msgs::Float64>("front_right_2_joint_position_controller/command",1);
//    ros::Publisher joint6ControllerCommandPub =
//            nh.advertise<std_msgs::Float64>("front_right_3_joint_position_controller/command",1);
//    ros::Publisher joint7ControllerCommandPub =
//            nh.advertise<std_msgs::Float64>("rear_right_1_joint_position_controller/command",1);
//    ros::Publisher joint8ControllerCommandPub =
//            nh.advertise<std_msgs::Float64>("rear_right_2_joint_position_controller/command",1);
//    ros::Publisher joint9ControllerCommandPub =
//            nh.advertise<std_msgs::Float64>("rear_right_3_joint_position_controller/command",1);
//    ros::Publisher joint10ControllerCommandPub =
//            nh.advertise<std_msgs::Float64>("rear_left_1_joint_position_controller/command",1);
//    ros::Publisher joint11ControllerCommandPub =
//            nh.advertise<std_msgs::Float64>("rear_left_2_joint_position_controller/command",1);
//    ros::Publisher joint12ControllerCommandPub =
//            nh.advertise<std_msgs::Float64>("rear_left_3_joint_position_controller/command",1);
    ros::Rate rate(50);
    ROS_INFO("get Ready");
    while(ros::ok())
    {
//            ROS_INFO("send joint command once");
            joint_group_position_pub.publish(joint_group_positions);

//        }

        ros::spinOnce();
        rate.sleep();

    }
}
