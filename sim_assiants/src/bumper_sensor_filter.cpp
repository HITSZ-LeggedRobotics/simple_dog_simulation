#include "ros/ros.h"
#include "gazebo_msgs/ContactsState.h"
#include "geometry_msgs/WrenchStamped.h"
#include "boost/thread.hpp"
#include "std_msgs/Bool.h"
#include "sim_assiants/FootContact.h"
#include "sim_assiants/FootContacts.h"

class BumperSensorFilter{
public:
  BumperSensorFilter(const ros::NodeHandle& node_handle)
    : node_handle_(node_handle)
  {
    foot_contacts_.foot_contacts.resize(4);
    // subcribe to bumper msg
    lf_bumperSensorSub_ = node_handle_.subscribe("/lf_foot_bumper", 1, &BumperSensorFilter::leftFrontFootBumperCallback, this);
    rf_bumperSensorSub_ = node_handle_.subscribe("/rf_foot_bumper", 1, &BumperSensorFilter::rightFrontFootBumperCallback, this);
    rh_bumperSensorSub_ = node_handle_.subscribe("/rh_foot_bumper", 1, &BumperSensorFilter::rightHindFootBumperCallback, this);
    lh_bumperSensorSub_ = node_handle_.subscribe("/lh_foot_bumper", 1, &BumperSensorFilter::leftHindFootBumperCallback, this);
    // publisher of contact force
    lf_footContactForcePub_ = node_handle_.advertise<geometry_msgs::WrenchStamped>("lf_foot_force", 1);
    rf_footContactForcePub_ = node_handle_.advertise<geometry_msgs::WrenchStamped>("rf_foot_force", 1);
    rh_footContactForcePub_ = node_handle_.advertise<geometry_msgs::WrenchStamped>("rh_foot_force", 1);
    lh_footContactForcePub_ = node_handle_.advertise<geometry_msgs::WrenchStamped>("lh_foot_force", 1);
    // publisher of contact state
    lf_footContactPub_ = node_handle_.advertise<sim_assiants::FootContact>("lf_foot_contact", 1);
    rf_footContactPub_ = node_handle_.advertise<sim_assiants::FootContact>("rf_foot_contact", 1);
    rh_footContactPub_ = node_handle_.advertise<sim_assiants::FootContact>("rh_foot_contact", 1);
    lh_footContactPub_ = node_handle_.advertise<sim_assiants::FootContact>("lh_foot_contact", 1);
    footContactsPub_ = node_handle_.advertise<sim_assiants::FootContacts>("foot_contacts", 1);
    // publish thread ON
    wrenchPublishThread_ = boost::thread(boost::bind(&BumperSensorFilter::wrenchPublishThreadFunction, this));
  }

private:
  /**
   * @brief leftFrontFootBumperCallback filter the information in Bumper msg
   * @param contact_state
   */
  void leftFrontFootBumperCallback(const gazebo_msgs::ContactsStateConstPtr& contact_state){
    lf_wrench_.header.frame_id = "lf_foot_Link";
    if(!contact_state->states.empty())
      {
        // if has contact, there same data in states array, just get the first
        lf_wrench_.wrench.force = contact_state->states[0].total_wrench.force;
        lf_wrench_.wrench.torque = contact_state->states[0].total_wrench.torque;
        lf_foot_contact_.is_contact = true;
        lf_foot_contact_.surface_normal.vector = contact_state->states[0].contact_normals[0];
        lf_foot_contact_.contact_position.vector = contact_state->states[0].contact_positions[0];
        lf_foot_contact_.contact_position.header.frame_id = "odom";
        lf_foot_contact_.name = "LF_LEG";
      } else{
        lf_wrench_.wrench.force.x = 0.0;
        lf_wrench_.wrench.force.y = 0.0;
        lf_wrench_.wrench.force.z = 0.0;
        lf_wrench_.wrench.torque.x = 0.0;
        lf_wrench_.wrench.torque.y = 0.0;
        lf_wrench_.wrench.torque.z = 0.0;

        lf_foot_contact_.is_contact = false;
      }
    lf_foot_contact_.contact_force.header = lf_wrench_.header;
    lf_foot_contact_.contact_force.wrench = lf_wrench_.wrench;
    foot_contacts_.foot_contacts[0] = lf_foot_contact_;

  }

  void rightFrontFootBumperCallback(const gazebo_msgs::ContactsStateConstPtr& contact_state)
  {
    rf_wrench_.header.frame_id = "rf_foot_Link";
    if(!contact_state->states.empty())
      {
        rf_wrench_.wrench.force = contact_state->states[0].total_wrench.force;
        rf_wrench_.wrench.torque = contact_state->states[0].total_wrench.torque;

        rf_foot_contact_.is_contact = true;
        rf_foot_contact_.surface_normal.vector = contact_state->states[0].contact_normals[0];
        rf_foot_contact_.contact_position.vector = contact_state->states[0].contact_positions[0];
        rf_foot_contact_.contact_position.header.frame_id = "odom";
        rf_foot_contact_.name = "RF_LEG";

      } else{
        rf_wrench_.wrench.force.x = 0.0;
        rf_wrench_.wrench.force.y = 0.0;
        rf_wrench_.wrench.force.z = 0.0;
        rf_wrench_.wrench.torque.x = 0.0;
        rf_wrench_.wrench.torque.y = 0.0;
        rf_wrench_.wrench.torque.z = 0.0;

        rf_foot_contact_.is_contact = false;
      }
    rf_foot_contact_.contact_force = rf_wrench_;
    foot_contacts_.foot_contacts[1] = rf_foot_contact_;

  }
  void rightHindFootBumperCallback(const gazebo_msgs::ContactsStateConstPtr& contact_state)
  {
    rh_wrench_.header.frame_id = "rh_foot_Link";
    if(!contact_state->states.empty())
      {
        rh_wrench_.wrench.force = contact_state->states[0].total_wrench.force;
        rh_wrench_.wrench.torque = contact_state->states[0].total_wrench.torque;

        rh_foot_contact_.is_contact = true;
        rh_foot_contact_.surface_normal.vector = contact_state->states[0].contact_normals[0];
        rh_foot_contact_.contact_position.vector = contact_state->states[0].contact_positions[0];
        rh_foot_contact_.contact_position.header.frame_id = "odom";
        rh_foot_contact_.name = "RH_LEG";

          } else{
        rh_wrench_.wrench.force.x = 0.0;
        rh_wrench_.wrench.force.y = 0.0;
        rh_wrench_.wrench.force.z = 0.0;
        rh_wrench_.wrench.torque.x = 0.0;
        rh_wrench_.wrench.torque.y = 0.0;
        rh_wrench_.wrench.torque.z = 0.0;

        rh_foot_contact_.is_contact = false;
      }
    rh_foot_contact_.contact_force = rh_wrench_;
    foot_contacts_.foot_contacts[2] = rh_foot_contact_;
  }
  void leftHindFootBumperCallback(const gazebo_msgs::ContactsStateConstPtr& contact_state)
  {
    lh_wrench_.header.frame_id = "lh_foot_Link";
    if(!contact_state->states.empty())
      {
        lh_wrench_.wrench.force = contact_state->states[0].total_wrench.force;
        lh_wrench_.wrench.torque = contact_state->states[0].total_wrench.torque;

        lh_foot_contact_.is_contact = true;
        lh_foot_contact_.surface_normal.vector = contact_state->states[0].contact_normals[0];
        lh_foot_contact_.contact_position.vector = contact_state->states[0].contact_positions[0];
        lh_foot_contact_.contact_position.header.frame_id = "odom";
        lh_foot_contact_.name = "LH_LEG";

          } else{
        lh_wrench_.wrench.force.x = 0.0;
        lh_wrench_.wrench.force.y = 0.0;
        lh_wrench_.wrench.force.z = 0.0;
        lh_wrench_.wrench.torque.x = 0.0;
        lh_wrench_.wrench.torque.y = 0.0;
        lh_wrench_.wrench.torque.z = 0.0;

        lh_foot_contact_.is_contact = false;
      }
    lh_foot_contact_.contact_force = lh_wrench_;
    foot_contacts_.foot_contacts[3] = lh_foot_contact_;
  }
  /**
   * @brief wrenchPublishThreadFunction
   */
  void wrenchPublishThreadFunction()
  {
    ros::Rate rate(50);
    while (ros::ok()) {
        // lock thread for data safe
        boost::recursive_mutex::scoped_lock lock(r_mutex_);
        lf_footContactForcePub_.publish(lf_wrench_);
        rf_footContactForcePub_.publish(rf_wrench_);
        rh_footContactForcePub_.publish(rh_wrench_);
        lh_footContactForcePub_.publish(lh_wrench_);

//        lf_footContactPub_.publish(lf_foot_contact_);
//        rf_footContactPub_.publish(rf_foot_contact_);
//        rh_footContactPub_.publish(rh_foot_contact_);
//        lh_footContactPub_.publish(lh_foot_contact_);
        footContactsPub_.publish(foot_contacts_);

        lock.unlock();
        rate.sleep();

      }
  }
  /**
   * @brief node_handle_
   */
  ros::NodeHandle node_handle_;
  /**
   * @brief lf_bumperSensorSub_
   */
  ros::Subscriber lf_bumperSensorSub_,rf_bumperSensorSub_,rh_bumperSensorSub_, lh_bumperSensorSub_;
  /**
   * @brief lf_footContactForcePub_
   */
  ros::Publisher lf_footContactForcePub_, rf_footContactForcePub_, rh_footContactForcePub_, lh_footContactForcePub_;
  /**
   * @brief lf_footContactPub_
   */
  ros::Publisher lf_footContactPub_, rf_footContactPub_, rh_footContactPub_, lh_footContactPub_;
  ros::Publisher footContactsPub_;
  /**
   * @brief lf_wrench_
   */
  geometry_msgs::WrenchStamped lf_wrench_, rf_wrench_, rh_wrench_, lh_wrench_;
  /**
   * @brief lf_foot_contact_
   */
  sim_assiants::FootContact lf_foot_contact_, rf_foot_contact_, rh_foot_contact_, lh_foot_contact_;
  sim_assiants::FootContacts foot_contacts_;
  /**
   * @brief wrenchPublishThread_
   */
  boost::thread wrenchPublishThread_;
  /**
   * @brief r_mutex_
   */
  boost::recursive_mutex r_mutex_;
};


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "bumper_sensor_filter_node");
  ros::NodeHandle node_handle("~");
  BumperSensorFilter bumperSensorFilter(node_handle);

  while (ros::ok()) {
      ros::spin();
    }
  return 0;
}
