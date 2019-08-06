# simple_dog_simulation
A simulation four leg robot simulation with gazebo

---

**modify by YaoChen-2019.8.5**
- modify the foot_Sensor message with 0/1, just for judeg;
- modify the pose_pub_node with the tf/topic name;
- modify the pose_pub_node with the gazebo/odom, use Odometry for compare;
- modify the pose_pub_node with the robot_state_;
- modify the launch wiht a cloister world in quadruped_world.launch;
---

## Build

```
cd your_catkin_ws/src
catkin_make
source ~/.bashrc
```
## Usage
- Launch dog model in a empty world
`roslaunch simpledog simpledog_empty_world.launch`
- Launch just RVIZ
`roslaunch simpledog simpledog_display.launch`
For the REAL quadruped
`roslaunch simpledog quadruped_display.launch`
- Launch interactive maker to send control target of leg
`roslaunch simpledog quadruped_interactive_marker.launch`

- Launch in a world with some obstacles
`roslaunch simpledog simpledog_simulation.launch`
**Attention:** open the test_terrain.world and search for `<model name='test_terrian_slope_and_stairs'>`, replace the STL model uri with you computer uri
- Pose Publisher
`rosrun sim_assiants pose_pub_node`
  > this is to publish the absolute base_link pose in world(odom frame), and TF, and a customed RobotState message

  >Node [/pose_pub_node]
  >> - Publications:
       * /gazebo/robot_states [free_gait_msgs/RobotState]

       * /pose_pub_node/base_pose [geometry_msgs/PoseWithCovarianceStamped]

       * /rosout [rosgraph_msgs/Log]

       * /tf [tf2_msgs/TFMessage]


  >>Subscriptions:
       * /clock [rosgraph_msgs/Clock]

       * /gazebo/model_states [gazebo_msgs/ModelStates]

       * /joint_states [sensor_msgs/JointState]

- Foot sensor
`rosrun sim_assiants bumper_sensor_filter_node`
  > this is to run a node to filter contact information from bumper sensor message

  >Node [/bumper_sensor_filter_node]
  >>Publications:
      * /bumper_sensor_filter_node/lf_foot_contact [sim_assiants/FootContact]

      * /bumper_sensor_filter_node/lf_foot_force [geometry_msgs/WrenchStamped]

      * /bumper_sensor_filter_node/lh_foot_contact [sim_assiants/FootContact]

      * /bumper_sensor_filter_node/lh_foot_force [geometry_msgs/WrenchStamped]

      * /bumper_sensor_filter_node/rf_foot_contact [sim_assiants/FootContact]

      * /bumper_sensor_filter_node/rf_foot_force [geometry_msgs/WrenchStamped]

      * /bumper_sensor_filter_node/rh_foot_contact [sim_assiants/FootContact]

      * /bumper_sensor_filter_node/rh_foot_force [geometry_msgs/WrenchStamped]

    >>Subscriptions:
      * /clock [rosgraph_msgs/Clock]

      * /lf_foot_bumper [gazebo_msgs/ContactsState]

      * /lh_foot_bumper [gazebo_msgs/ContactsState]

      * /rf_foot_bumper [gazebo_msgs/ContactsState]

      * /rh_foot_bumper [gazebo_msgs/ContactsState]

### How to control the robot
Publish ROS msg `sensor_msgs/JointStates` topic:
joint position array of 12 elements
- 0 -> 2 front left leg
- 3 -> 5 front right leg
- 6 -> 8 hind right leg
- 9 -> 11 hind left leg

```
ros::Publisher joint1ControllerCommandPub =
        nh.advertise<std_msgs::Float64>("front_left_1_joint_position_controller/command",1);
ros::Publisher joint2ControllerCommandPub =
        nh.advertise<std_msgs::Float64>("front_left_2_joint_position_controller/command",1);
ros::Publisher joint3ControllerCommandPub =
        nh.advertise<std_msgs::Float64>("front_left_3_joint_position_controller/command",1);
ros::Publisher joint4ControllerCommandPub =
        nh.advertise<std_msgs::Float64>("front_right_1_joint_position_controller/command",1);
ros::Publisher joint5ControllerCommandPub =
        nh.advertise<std_msgs::Float64>("front_right_2_joint_position_controller/command",1);
ros::Publisher joint6ControllerCommandPub =
        nh.advertise<std_msgs::Float64>("front_right_3_joint_position_controller/command",1);
ros::Publisher joint7ControllerCommandPub =
        nh.advertise<std_msgs::Float64>("rear_right_1_joint_position_controller/command",1);
ros::Publisher joint8ControllerCommandPub =
        nh.advertise<std_msgs::Float64>("rear_right_2_joint_position_controller/command",1);
ros::Publisher joint9ControllerCommandPub =
        nh.advertise<std_msgs::Float64>("rear_right_3_joint_position_controller/command",1);
ros::Publisher joint10ControllerCommandPub =
        nh.advertise<std_msgs::Float64>("rear_left_1_joint_position_controller/command",1);
ros::Publisher joint11ControllerCommandPub =
        nh.advertise<std_msgs::Float64>("rear_left_2_joint_position_controller/command",1);
ros::Publisher joint12ControllerCommandPub =
        nh.advertise<std_msgs::Float64>("rear_left_3_joint_position_controller/command",1);
```


There also joint feed back,
`rostopic list`
to see some feedback topic
