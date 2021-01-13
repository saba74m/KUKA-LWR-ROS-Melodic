This file consist of many packages:

1. Kuka LWR 4+ Description Package:
This pkg tend to implement the robot model by means of URDF and xacro files. 

To see that URDF implemented correctly, run:
roslaunch kuka_description urdf_visualize.launch

2. KUKA LWR 4+ MoveIt Package:

To visulize without any error:
ROS Melodic + UBUNTU 18.04 is advised.

To visulize and start planning, simply run:
roslaunch kuka_moveit demo.launch

3. PathFollowing Package (ToDo)
This can generally work with all kinds of robots

4. Kuka LWR 4+ controllers (ToDo)

5. Haptic Device controllers (ToDo)

Other repositories included:

1. Bag: To include all the recorded bag files from Omni Haptic Device on /omni1_pose topic =>> message type: geometry_msgs/PoseStamped

2. task_description: for now the abdomen model and the wire buzz game added, the later needs to be modified and not complete yet.
