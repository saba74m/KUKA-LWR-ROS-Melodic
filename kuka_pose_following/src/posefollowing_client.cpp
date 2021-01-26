#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "kuka_pose_following/MoveToQuat.h"
#include <math.h>

/* Written By Saba Mansourfar
/ Politecnico Di Milano

This Node is responsible for 
1. Receiving the positions and orientations
2. Delivering it to Service "Server"

**Important Notice: In order to decrease the log file volume, "ROS INFO" is commented.
UNCOMMENT the "ROS INFO" lines, in the case you want to see the full conversions process.*/

class pose_sub
{
  public:
  pose_sub(){

    //Subscribing to the OMNI_POSE topic
    sub=n.subscribe("/omni1_pose",1000, &pose_sub::posefollowCallback,this);

    //Create service client
    client = n.serviceClient<kuka_pose_following::MoveToQuat>("/moveit_interface/move_to_quat");

  }

  //Callback Function for "Omni"
  void posefollowingCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  //ROS_INFO("CALLBACK IS WORKING")

  //temporarily parameters to save the "msg"
  float temp_px, temp_py, temp_pz;
  float temp_ox, temp_oy, temp_oz, temp_ow;

  //Position of the Omni
  temp_px = msg->pose.position.x;
  temp_py = msg->pose.position.y;
  temp_pz = msg->pose.position.z;

  //Orientation of the Omni
  temp_ox = msg->pose.orientation.x;
  temp_oy = msg->pose.orientation.y;
  temp_oz = msg->pose.orientation.z;
  temp_ow = msg->pose.orientation.z;
  ROS_INFO("Position: (%f, %f, %f) and Orientation: (%f, %f, %f) of the omni", temp_px,  temp_py,  temp_pz,  temp_ox, temp_oy, temp_oz);

  //Client Request of Position
  srv.request.pose_quat[0]= temp_px;
  srv.request.pose_quat[1]= temp_py;
  srv.request.pose_quat[2]= temp_pz;

  //Client Request of Orientation
  srv.request.pose_quat[3]= temp_ox;
  srv.request.pose_quat[4]= temp_oy;
  srv.request.pose_quat[5]= temp_oz;
  srv.request.pose_quat[6]= temp_ow;

  srv.request.max_vel_fact=0.1;
  srv.request.max_acc_fact=0.1;

  //Calling Service "srv" over Client
  client.call(srv);


  }
  
  
  //Creating Node Handle, Subscriber, Service
  private:
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Publisher odom_pub;
  ros::ServiceClient client;
  geometry_msgs::PoseStamped pose_stamped;     //Creating "/OMNI1_POSE" vector
  kuka_pose_following::MoveToQuat srv;  //Conncting to the "MeveQuat" file in the package
 
};


int main(int argc, char** argv){
  
  //Initializing the Node "posefollowing_client"
  ros::init(argc, argv, "posefollowing_client");

  //calling the class "pose_sub"
  pose_sub pose_sub;

  
  ros::spin();
  return 0;
}

