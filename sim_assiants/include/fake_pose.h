#ifndef FAKE_POSE_H
#define FAKE_POSE_H
#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "gazebo_msgs/ModelStates.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"
#include "boost/thread.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_sequencer.h"
#include "free_gait_msgs/RobotState.h"
#include "sim_assiants/FootContacts.h"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Core"

namespace fake_pose {

class FakePose
{
public:

    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    FakePose(ros::NodeHandle& nodeHandle);

    /*!
     * Destructor.
     */
    virtual ~FakePose();

    /*!
     * Callback function for get model states from gazebo and pub as a absolute locate.
     * @param modelStates is a gazebo message containing position and orientation.
     */
    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& modelStatesMsg);

    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& joint_states);

    void footContactsCallback(const sim_assiants::FootContacts::ConstPtr& foot_contacts);
private:
    //! subscribe loop thread
    void modelStatesSubLoopThread();

    //! ROS nodehandle
    ros::NodeHandle& nodeHandle_;

    //! ROS subscriber
    ros::Subscriber modelStatesSub_, gazebo_joint_states_sub_, footContactsSub_;

    //! message filter subscriber
    //message_filters::Subscriber<gazebo_msgs::ModelStates> timeSeqSub_;

    //! ROS publisher
    ros::Publisher fakePosePub_, robot_state_pub_;

    //! pose
    geometry_msgs::PoseWithCovarianceStamped fakePoseMsg_;

    //! TF boardcaster
    tf::TransformBroadcaster tfBoardcaster_;

    //! boost thread
    boost::thread modelStatesSubLoopThread_;

    free_gait_msgs::RobotState robot_state_;

    boost::recursive_mutex r_mutex_;

    tf::Transform odom2base, odom_to_footprint, footprint_to_base;
    tf::Quaternion q;
    ros::Time gazebo_time;

};

}/* namespace */
#endif // FAKE_POSE_H
