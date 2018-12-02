# simple_dog_simulation
A simulation four leg robot simulation with gazebo
**TODO(Shunyao): add a foot contact sensor**
## Build
```
cd your_catkin_ws/src
catkin_make
source ~/.bashrc
```
## Usage
- Launch dog model in a empty world
`roslaunch simpledog simpledog_empty_world.launch`
- Launch in a world with some obstacles
`roslaunch simpledog simple_dog_simulation.launch`

### How to control the robot
Publish ROS msg std_msgs/Float64, topic name as follow:
> front_left_1_joint_position_controller/command
front_left_2_joint_position_controller/command
front_left_3_joint_position_controller/command
front_right_1_joint_position_controller/command
front_right_2_joint_position_controller/command
front_right_3_joint_position_controller/command
rear_right_1_joint_position_controller/command
rear_right_2_joint_position_controller/command
rear_right_3_joint_position_controller/command
rear_left_1_joint_position_controller/command
rear_left_2_joint_position_controller/command
rear_left_3_joint_position_controller/command

There also joint feed back,
`rostopic list`
to see some feedback topic
