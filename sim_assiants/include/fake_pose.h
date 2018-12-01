#ifndef FAKE_POSE_H
#define FAKE_POSE_H
#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "gazebo_msgs/ModelStates.h"
#include "tf/transform_broadcaster.h"
#include "boost/thread.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_sequencer.h"

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

private:
    //! subscribe loop thread
    void modelStatesSubLoopThread();

    //! ROS nodehandle
    ros::NodeHandle& nodeHandle_;

    //! ROS subscriber
    ros::Subscriber modelStatesSub_;

    //! message filter subscriber
    //message_filters::Subscriber<gazebo_msgs::ModelStates> timeSeqSub_;

    //! ROS publisher
    ros::Publisher fakePosePub_;

    //! pose
    geometry_msgs::PoseWithCovarianceStamped fakePoseMsg_;

    //! TF boardcaster
    tf::TransformBroadcaster tfBoardcaster_;

    //! boost thread
    boost::thread modelStatesSubLoopThread_;
};

}/* namespace */
#endif // FAKE_POSE_H
