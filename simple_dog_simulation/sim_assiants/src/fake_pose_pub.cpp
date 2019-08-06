#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_listener.h"
#include "gazebo_msgs/ModelStates.h"
#include "boost/bind.hpp"
#include "free_gait_msgs/RobotState.h"
#include "sim_assiants/FootContacts.h"
#include "fake_pose.h"

namespace fake_pose {

FakePose::FakePose(ros::NodeHandle& nodehandle)
    : nodeHandle_(nodehandle)
{
    ROS_INFO("constructing.....");
    modelStatesSub_ = nodeHandle_.subscribe("/gazebo/model_states", 1, &FakePose::modelStatesCallback, this);
    gazebo_joint_states_sub_ = nodeHandle_.subscribe("/joint_states", 1, &FakePose::jointStatesCallback, this);
    footContactsSub_ = nodeHandle_.subscribe("/bumper_sensor_filter_node/foot_contacts", 1, &FakePose::footContactsCallback, this);
    fakePosePub_ = nodeHandle_.advertise<geometry_msgs::PoseWithCovarianceStamped>("base_pose", 1);
    robot_state_pub_ = nodeHandle_.advertise<free_gait_msgs::RobotState>("/gazebo/robot_states", 1);
    gazebo_pub = nodeHandle_.advertise<nav_msgs::Odometry>("/gazebo/odom",1);

    robot_state_.lf_leg_joints.name.resize(3);
    robot_state_.lf_leg_joints.position.resize(3);
    robot_state_.lf_leg_joints.velocity.resize(3);
    robot_state_.lf_leg_joints.effort.resize(3);

    robot_state_.rf_leg_joints.name.resize(3);
    robot_state_.rf_leg_joints.position.resize(3);
    robot_state_.rf_leg_joints.velocity.resize(3);
    robot_state_.rf_leg_joints.effort.resize(3);

    robot_state_.lh_leg_joints.name.resize(3);
    robot_state_.lh_leg_joints.position.resize(3);
    robot_state_.lh_leg_joints.velocity.resize(3);
    robot_state_.lh_leg_joints.effort.resize(3);

    robot_state_.rh_leg_joints.name.resize(3);
    robot_state_.rh_leg_joints.position.resize(3);
    robot_state_.rh_leg_joints.velocity.resize(3);
    robot_state_.rh_leg_joints.effort.resize(3);
    //    message_filters::Subscriber<gazebo_msgs::ModelStates> timeSeqSub_(nodeHandle_,"/gazebo/model_states", 1);
//    message_filters::TimeSequencer<gazebo_msgs::ModelStates> seq(timeSeqSub_, ros::Duration(0.1), ros::Duration(0.01),10,nodeHandle_);
//    seq.registerCallback(&FakePose::modelStatesCallback,this);

    //timeSeqSub_ = message_filters::Subscriber<gazebo_msgs::ModelStates>("/gazebo/model_states",1);
     modelStatesSubLoopThread_ = boost::thread(boost::bind(&FakePose::modelStatesSubLoopThread, this));

}

FakePose::~FakePose(){};

void FakePose::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& joint_states)
{
  gazebo_time = joint_states->header.stamp;
  robot_state_.lf_leg_joints.header = joint_states->header;
  robot_state_.lf_leg_joints.name[0] = "front_left_1_joint";
  robot_state_.lf_leg_joints.position[0] = joint_states->position[0];
  robot_state_.lf_leg_joints.velocity[0] = joint_states->velocity[0];
  robot_state_.lf_leg_joints.effort[0] = joint_states->effort[0];
  robot_state_.lf_leg_joints.name[1] = "front_left_2_joint";
  robot_state_.lf_leg_joints.position[1] = joint_states->position[1];
  robot_state_.lf_leg_joints.velocity[1] = joint_states->velocity[1];
  robot_state_.lf_leg_joints.effort[1] = joint_states->effort[1];
  robot_state_.lf_leg_joints.name[2] = "front_left_3_joint";
  robot_state_.lf_leg_joints.position[2] = joint_states->position[2];
  robot_state_.lf_leg_joints.velocity[2] = joint_states->velocity[2];
  robot_state_.lf_leg_joints.effort[2] = joint_states->effort[2];

  robot_state_.rf_leg_joints.header = joint_states->header;
  robot_state_.rf_leg_joints.name[0] = "front_right_1_joint";
  robot_state_.rf_leg_joints.position[0] = joint_states->position[3];
  robot_state_.rf_leg_joints.velocity[0] = joint_states->velocity[3];
  robot_state_.rf_leg_joints.effort[0] = joint_states->effort[3];
  robot_state_.rf_leg_joints.name[1] = "front_right_2_joint";
  robot_state_.rf_leg_joints.position[1] = joint_states->position[4];
  robot_state_.rf_leg_joints.velocity[1] = joint_states->velocity[4];
  robot_state_.rf_leg_joints.effort[1] = joint_states->effort[4];
  robot_state_.rf_leg_joints.name[2] = "front_right_3_joint";
  robot_state_.rf_leg_joints.position[2] = joint_states->position[5];
  robot_state_.rf_leg_joints.velocity[2] = joint_states->velocity[5];
  robot_state_.rf_leg_joints.effort[2] = joint_states->effort[5];

  robot_state_.lh_leg_joints.header = joint_states->header;
  robot_state_.lh_leg_joints.name[0] = "rear_left_1_joint";
  robot_state_.lh_leg_joints.position[0] = joint_states->position[6];
  robot_state_.lh_leg_joints.velocity[0] = joint_states->velocity[6];
  robot_state_.lh_leg_joints.effort[0] = joint_states->effort[6];
  robot_state_.lh_leg_joints.name[1] = "rear_left_2_joint";
  robot_state_.lh_leg_joints.position[1] = joint_states->position[7];
  robot_state_.lh_leg_joints.velocity[1] = joint_states->velocity[7];
  robot_state_.lh_leg_joints.effort[1] = joint_states->effort[7];
  robot_state_.lh_leg_joints.name[2] = "rear_left_3_joint";
  robot_state_.lh_leg_joints.position[2] = joint_states->position[8];
  robot_state_.lh_leg_joints.velocity[2] = joint_states->velocity[8];
  robot_state_.lh_leg_joints.effort[2] = joint_states->effort[8];

  robot_state_.rh_leg_joints.header = joint_states->header;
  robot_state_.rh_leg_joints.name[0] = "rear_right_1_joint";
  robot_state_.rh_leg_joints.position[0] = joint_states->position[9];
  robot_state_.rh_leg_joints.velocity[0] = joint_states->velocity[9];
  robot_state_.rh_leg_joints.effort[0] = joint_states->effort[9];
  robot_state_.rh_leg_joints.name[1] = "rear_right_2_joint";
  robot_state_.rh_leg_joints.position[1] = joint_states->position[10];
  robot_state_.rh_leg_joints.velocity[1] = joint_states->velocity[10];
  robot_state_.rh_leg_joints.effort[1] = joint_states->effort[10];
  robot_state_.rh_leg_joints.name[2] = "rear_right_3_joint";
  robot_state_.rh_leg_joints.position[2] = joint_states->position[11];
  robot_state_.rh_leg_joints.velocity[2] = joint_states->velocity[11];
  robot_state_.rh_leg_joints.effort[2] = joint_states->effort[11];
}

void FakePose::footContactsCallback(const sim_assiants::FootContacts::ConstPtr& foot_contacts)
{
  robot_state_.lf_leg_mode.support_leg = foot_contacts->foot_contacts[0].is_contact;
  robot_state_.lf_leg_mode.name = foot_contacts->foot_contacts[0].name;
  robot_state_.lf_leg_mode.surface_normal = foot_contacts->foot_contacts[0].surface_normal;

  robot_state_.rf_leg_mode.support_leg = foot_contacts->foot_contacts[1].is_contact;
  robot_state_.rf_leg_mode.name = foot_contacts->foot_contacts[1].name;
  robot_state_.rf_leg_mode.surface_normal = foot_contacts->foot_contacts[1].surface_normal;

  robot_state_.rh_leg_mode.support_leg = foot_contacts->foot_contacts[2].is_contact;
  robot_state_.rh_leg_mode.name = foot_contacts->foot_contacts[2].name;
  robot_state_.rh_leg_mode.surface_normal = foot_contacts->foot_contacts[2].surface_normal;

  robot_state_.lh_leg_mode.support_leg = foot_contacts->foot_contacts[3].is_contact;
  robot_state_.lh_leg_mode.name = foot_contacts->foot_contacts[3].name;
  robot_state_.lh_leg_mode.surface_normal = foot_contacts->foot_contacts[3].surface_normal;
}

void FakePose::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& modelStatesMsg)
{
    ROS_INFO("Recieved a model states");
//    geometry_msgs::Pose base_pose = *(modelStatesMsg->pose.end());
//    geometry_msgs::Twist base_twist =*(modelStatesMsg->twist.end());
      geometry_msgs::Pose base_pose = modelStatesMsg->pose[10];
      geometry_msgs::Twist base_twist =modelStatesMsg->twist[10];
    fakePoseMsg_.pose.pose = base_pose;
    robot_state_.base_pose.pose = fakePoseMsg_.pose;
    robot_state_.base_pose.child_frame_id = "/base_link";
    robot_state_.base_pose.twist.twist = base_twist;
    robot_state_.base_pose.header.frame_id = "/odom";

    gazeboPoseMsg.header.stamp = ros::Time::now();
    gazeboPoseMsg.pose.pose = base_pose;
    gazeboPoseMsg.twist.twist = base_twist;
    gazeboPoseMsg.header.frame_id = "/gazebo_odom";
    gazeboPoseMsg.child_frame_id = "/gazebo_baselink";


    q.setW(base_pose.orientation.w);
    q.setX(base_pose.orientation.x);
    q.setY(base_pose.orientation.y);
    q.setZ(base_pose.orientation.z);
    odom2base.setRotation(q);
    odom2base.setOrigin(tf::Vector3(base_pose.position.x,
                                    base_pose.position.y,
                                    base_pose.position.z));


    double yaw, pitch, roll;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    odom_to_footprint.setRotation(tf::createQuaternionFromYaw(yaw));
    odom_to_footprint.setOrigin(tf::Vector3(base_pose.position.x,
                                base_pose.position.y,
                                0));

    footprint_to_base.setRotation(tf::createQuaternionFromRPY(roll, pitch, 0.0));
    footprint_to_base.setOrigin(tf::Vector3(0,0,base_pose.position.z));

//    tfBoardcaster_.sendTransform(tf::StampedTransform(odom2base, ros::Time::now(), "/odom", "/base_link"));
//    fakePosePub_.publish(fakePoseMsg_);
    // ros::Duration(0.01).sleep();
}

void FakePose::modelStatesSubLoopThread()
{
    static const double timeout = 0.02;
    ros::Rate rate(100);
    while(nodeHandle_.ok())
    {
        boost::recursive_mutex::scoped_lock lock(r_mutex_);
        robot_state_pub_.publish(robot_state_);
//        tfBoardcaster_.sendTransform(tf::StampedTransform(odom2base, ros::Time::now(), "/odom", "/base_link"));

        tfBoardcaster_.sendTransform(tf::StampedTransform(odom_to_footprint, gazebo_time, "/odom", "/foot_print"));
        tfBoardcaster_.sendTransform(tf::StampedTransform(footprint_to_base, gazebo_time, "foot_print","/base_link"));
        fakePosePub_.publish(fakePoseMsg_);
        gazebo_pub.publish(gazeboPoseMsg);
        lock.unlock();
        ROS_INFO("in test thread");

        rate.sleep();
//        ros::spinOnce();
//        rate.sleep();

    }
}

}
