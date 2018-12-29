#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_listener.h"
#include "gazebo_msgs/ModelStates.h"
#include "fake_pose.h"
#include "boost/bind.hpp"

namespace fake_pose {

FakePose::FakePose(ros::NodeHandle& nodehandle)
    : nodeHandle_(nodehandle)
{
    ROS_INFO("constructing.....");
    modelStatesSub_ = nodeHandle_.subscribe("/gazebo/model_states", 1, &FakePose::modelStatesCallback, this);
    fakePosePub_ = nodeHandle_.advertise<geometry_msgs::PoseWithCovarianceStamped>("base_pose",1);
//    message_filters::Subscriber<gazebo_msgs::ModelStates> timeSeqSub_(nodeHandle_,"/gazebo/model_states", 1);
//    message_filters::TimeSequencer<gazebo_msgs::ModelStates> seq(timeSeqSub_, ros::Duration(0.1), ros::Duration(0.01),10,nodeHandle_);
//    seq.registerCallback(&FakePose::modelStatesCallback,this);

    //timeSeqSub_ = message_filters::Subscriber<gazebo_msgs::ModelStates>("/gazebo/model_states",1);
    // modelStatesSubLoopThread_ = boost::thread(boost::bind(&FakePose::modelStatesSubLoopThread, this));
}

FakePose::~FakePose(){};

void FakePose::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& modelStatesMsg)
{
    ROS_INFO("Recieved a model states");
    fakePoseMsg_.pose.pose = modelStatesMsg->pose[9];

    tf::Transform odom2base;
    tf::Quaternion q;
    q.setW(modelStatesMsg->pose[9].orientation.w);
    q.setX(modelStatesMsg->pose[9].orientation.x);
    q.setY(modelStatesMsg->pose[9].orientation.y);
    q.setZ(modelStatesMsg->pose[9].orientation.z);
    odom2base.setRotation(q);

    odom2base.setOrigin(tf::Vector3(modelStatesMsg->pose[9].position.x,
                                    modelStatesMsg->pose[9].position.y,
                                    modelStatesMsg->pose[9].position.z));
    tfBoardcaster_.sendTransform(tf::StampedTransform(odom2base, ros::Time::now(), "/odom", "/base_link"));


    fakePosePub_.publish(fakePoseMsg_);
    // ros::Duration(0.01).sleep();
}

void FakePose::modelStatesSubLoopThread()
{
    static const double timeout = 0.02;
    ros::Rate rate(100);
    while(nodeHandle_.ok())
    {
        ROS_INFO("in test thread");
        rate.sleep();
//        ros::spinOnce();
//        rate.sleep();
        //ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(timeout));
    }
}

}
