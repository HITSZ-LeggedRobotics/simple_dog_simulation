/*
 * fake_pose_node.cpp
 *
 *  Created on: Oct 6, 2018
 *      Author: Shunyao Wang
 *   Institute: HIT Shenzhen
 */

#include <ros/ros.h>
#include "fake_pose.h"
//#include "sim_assiants/fake_pose.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_pose_node");
    ros::NodeHandle nodehandle("~");
    fake_pose::FakePose fakepose(nodehandle);

    ROS_INFO("Hello world!");
//    ros::Rate rate(50);
    while(ros::ok())
    {
        ros::spin();
    }
//    ros::AsyncSpinner spinner(1);
//    spinner.start();
//    ros::waitForShutdown();
    return 0;
}
