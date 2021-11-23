Thanks to the @CentroEPiaggio group (https://github.com/CentroEPiaggio/kuka-lwr) for making this project public, so I can adjust it depending on my system and my controllers
____________  _________  ___________  _______________________________________________________
_____________  __________  ___________  _____________________________________________________
Before using this file make sure you have already installed the prerequested packages:
MoveIt, joint_state_publisher, robot_state_publisher, ros_control, ... 

This file consist of many packages:

1. Kuka LWR 4+ Description Package:
This pkg tend to implement the robot model by means of URDF and xacro files. 

To see that URDF implemented correctly, run:
roslaunch kuka_description urdf_rviz.launch

2. KUKA LWR 4+ MoveIt Package:

To visulize without any error:
ROS Melodic + UBUNTU 18.04 is advised.

To visulize and start planning, simply run:
roslaunch kuka_robot kuka_robot

3. PathFollowing Package 
This can generally work with all kinds of robots

4. Kuka LWR 4+ controllers
Add the specific controllers from RosControl package and some specific controllers for KUKA. (Debugging Phase)
For starting the controller(s):
...

5. Haptic Device controllers (ToDo)
