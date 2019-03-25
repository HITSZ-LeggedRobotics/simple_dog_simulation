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
    gazebo_joint_states_sub_ = nodeHandle_.subscribe("/joint_states", 1, &FakePose::jointStatesCallback, this);
    footContactsSub_ = nodeHandle_.subscribe("/bumper_sensor_filter_node/foot_contacts", 1, &FakePose::footContactsCallback, this);
    fakePosePub_ = nodeHandle_.advertise<geometry_msgs::PoseWithCovarianceStamped>("base_pose", 1);
    robot_state_pub_ = nodeHandle_.advertise<free_gait_msgs::RobotState>("/gazebo/robot_states", 1);

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
  robot_state_.lf_leg_joints.header = joint_states->header;
  robot_state_.lf_leg_joints.name[0] = "front_left_1_joint";
  robot_state_.lf_leg_joints.position[0] = joint_states->position[0];
  robot_state_.lf_leg_joints.effort[0] = joint_states->effort[0];
  robot_state_.lf_leg_joints.name[1] = "front_left_2_joint";
  robot_state_.lf_leg_joints.position[1] = joint_states->position[1];
  robot_state_.lf_leg_joints.effort[1] = joint_states->effort[1];
  robot_state_.lf_leg_joints.name[2] = "front_left_3_joint";
  robot_state_.lf_leg_joints.position[2] = joint_states->position[2];
  robot_state_.lf_leg_joints.effort[2] = joint_states->effort[2];

  robot_state_.rf_leg_joints.header = joint_states->header;
  robot_state_.rf_leg_joints.name[0] = "front_right_1_joint";
  robot_state_.rf_leg_joints.position[0] = joint_states->position[3];
  robot_state_.rf_leg_joints.effort[0] = joint_states->effort[3];
  robot_state_.rf_leg_joints.name[1] = "front_right_2_joint";
  robot_state_.rf_leg_joints.position[1] = joint_states->position[4];
  robot_state_.rf_leg_joints.effort[1] = joint_states->effort[4];
  robot_state_.rf_leg_joints.name[2] = "front_right_3_joint";
  robot_state_.rf_leg_joints.position[2] = joint_states->position[5];
  robot_state_.rf_leg_joints.effort[2] = joint_states->effort[5];

  robot_state_.lh_leg_joints.header = joint_states->header;
  robot_state_.lh_leg_joints.name[0] = "rear_left_1_joint";
  robot_state_.lh_leg_joints.position[0] = joint_states->position[6];
  robot_state_.lh_leg_joints.effort[0] = joint_states->effort[6];
  robot_state_.lh_leg_joints.name[1] = "rear_left_2_joint";
  robot_state_.lh_leg_joints.position[1] = joint_states->position[7];
  robot_state_.lh_leg_joints.effort[1] = joint_states->effort[7];
  robot_state_.lh_leg_joints.name[2] = "rear_left_3_joint";
  robot_state_.lh_leg_joints.position[2] = joint_states->position[8];
  robot_state_.lh_leg_joints.effort[2] = joint_states->effort[8];

  robot_state_.rh_leg_joints.header = joint_states->header;
  robot_state_.rh_leg_joints.name[0] = "rear_right_1_joint";
  robot_state_.rh_leg_joints.position[0] = joint_states->position[9];
  robot_state_.rh_leg_joints.effort[0] = joint_states->effort[9];
  robot_state_.rh_leg_joints.name[1] = "rear_right_2_joint";
  robot_state_.rh_leg_joints.position[1] = joint_states->position[10];
  robot_state_.rh_leg_joints.effort[1] = joint_states->effort[10];
  robot_state_.rh_leg_joints.name[2] = "rear_right_3_joint";
  robot_state_.rh_leg_joints.position[2] = joint_states->position[11];
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
    fakePoseMsg_.pose.pose = modelStatesMsg->pose[9];
    robot_state_.base_pose.pose = fakePoseMsg_.pose;
    robot_state_.base_pose.child_frame_id = "/base_link";
    robot_state_.base_pose.twist.twist = modelStatesMsg->twist[9];
    robot_state_.base_pose.header.frame_id = "/odom";

//    tf::Transform odom2base;
//    tf::Quaternion q;
    q.setW(modelStatesMsg->pose[9].orientation.w);
    q.setX(modelStatesMsg->pose[9].orientation.x);
    q.setY(modelStatesMsg->pose[9].orientation.y);
    q.setZ(modelStatesMsg->pose[9].orientation.z);
    odom2base.setRotation(q);

    odom2base.setOrigin(tf::Vector3(modelStatesMsg->pose[9].position.x,
                                    modelStatesMsg->pose[9].position.y,
                                    modelStatesMsg->pose[9].position.z));
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
        tfBoardcaster_.sendTransform(tf::StampedTransform(odom2base, ros::Time::now(), "/odom", "/base_link"));
        fakePosePub_.publish(fakePoseMsg_);
        lock.unlock();
        ROS_INFO("in test thread");

        rate.sleep();
//        ros::spinOnce();
//        rate.sleep();
        //ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(timeout));
    }
}

}
