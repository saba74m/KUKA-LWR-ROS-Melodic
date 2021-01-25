#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include "manipulator_pose_following/DeltaPoseRPY.h"
#include "manipulator_pose_following/ReplyInt.h"

#include <math.h>

// --- Globals
bool g_do_pose_following = false;
const int STATE_IDLE = 0;
const int STATE_POSE_FOLLOW = 1;
const int STATE_STOP = 2;
int g_state = STATE_IDLE;
sensor_msgs::JointState g_current_joints;
ros::Time g_t_start, g_t_last, g_t_last_cb;
// TODO Make below message a stamped one
manipulator_pose_following::DeltaPoseRPY g_delta_pose;
geometry_msgs::PoseStamped g_pose, g_pose_last;

Eigen::MatrixXd p_inverse(Eigen::MatrixXd J) {
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
  return (J.transpose() * (J * J.transpose()).inverse());
}

Eigen::MatrixXd sr_inverse(Eigen::MatrixXd J, double w, double w0, double k0) {
  double k = 0;
  if (w < w0) {
    k = k0 * pow(1 - w / w0, 2);
    ROS_DEBUG_NAMED("stream_J_cnd", "k: %e", k);
  }

  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
  return (J.transpose() * (J * J.transpose() + k * I).inverse());
}

void cb_joint_state(sensor_msgs::JointState msg) {
  //ROS_DEBUG_NAMED("stream_dbg", "New joint state...");
  g_current_joints = msg;
}

bool cb_start_pose_following(
    manipulator_pose_following::ReplyInt::Request &req,
    manipulator_pose_following::ReplyInt::Response &res) {

  ROS_INFO("/pose_following/start signal received.");
  g_do_pose_following = true;
  g_state = STATE_IDLE;
  ROS_DEBUG_NAMED("state", "STATE_STOP --> STATE_IDLE");
  ROS_DEBUG_NAMED("event", "Event: received start signal");
  res.reply = 0;
  return 0;
}

bool cb_stop_pose_following(
    manipulator_pose_following::ReplyInt::Request &req,
    manipulator_pose_following::ReplyInt::Response &res) {

  ROS_INFO("/pose_following/stop signal received.");
  g_do_pose_following = false;
  g_state = STATE_STOP;
  ROS_DEBUG_NAMED("state", "--> STATE_STOP");
  ROS_DEBUG_NAMED("event", "Event: received stop signal");
  res.reply = 0;
  return 0;
}

void cb_delta_pose_rpy(const geometry_msgs::Twist::ConstPtr &msg) {

  if (g_state == STATE_IDLE) {
    g_state = STATE_POSE_FOLLOW;
    ROS_DEBUG_NAMED("state", "STATE_IDLE --> STATE_POSE_FOLLOW");
    ROS_DEBUG_NAMED("event", "Event: received msg on topic /cmd_vel");
    g_t_start = ros::Time::now();
  }

  g_t_last_cb = ros::Time::now();
  // Check valid deltas:
  /*if (msg->data.size() != 6) {
    ROS_ERROR("Delta pose must be of size 6 (position, orientation (RPY)). "
              "Ignoring message.");
    return;
  }*/
  // TODO make 'DeltaPoseRPY' a stamped message (header) to merge the timestamp
  //      g_t_last_cb into it.
  g_delta_pose.data.clear();
  g_delta_pose.data.resize(6, 0);
  g_delta_pose.data.at(0) = msg->linear.x;
  g_delta_pose.data.at(1) = msg->linear.y;
  g_delta_pose.data.at(2) = msg->linear.z;
  g_delta_pose.data.at(3) = msg->angular.x;
  g_delta_pose.data.at(4) = msg->angular.y;
  g_delta_pose.data.at(5) = msg->angular.z;
  // g_delta_pose = *msg;
}

manipulator_pose_following::DeltaPoseRPY
calc_dpose(geometry_msgs::PoseStamped pose) {

  ROS_DEBUG_STREAM_NAMED("stream_calc_dpose", "calc_dpose:pose: \n" << pose);

  ros::Time t_start = ros::Time::now();
  // TODO the initial |dt| might be really hight -> add a check on it
  double dt = 0;
  manipulator_pose_following::DeltaPoseRPY delta_pose;
  manipulator_pose_following::DeltaPoseRPY pose_rpy, pose_rpy_last;
  pose_rpy.data.resize(6, 0);
  pose_rpy_last.data.resize(6, 0);
  delta_pose.data.resize(6, 0);

  if (g_pose_last.header.stamp.toSec() != 0) {
    dt = (pose.header.stamp - g_pose_last.header.stamp).toSec();

    // --- Convert from quaternions to Euler angles
    tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y,
                     pose.pose.orientation.z, pose.pose.orientation.w);
    tf::Quaternion q_last(
        g_pose_last.pose.orientation.x, g_pose_last.pose.orientation.y,
        g_pose_last.pose.orientation.z, g_pose_last.pose.orientation.w);
    tf::Matrix3x3 m(q);
    tf::Matrix3x3 m_last(q_last);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    pose_rpy.data[3] = roll;
    pose_rpy.data[4] = pitch;
    pose_rpy.data[5] = yaw;
    m_last.getRPY(roll, pitch, yaw);
    pose_rpy_last.data[3] = roll;
    pose_rpy_last.data[4] = pitch;
    pose_rpy_last.data[5] = yaw;
    pose_rpy.data[0] = pose.pose.position.x;
    pose_rpy.data[1] = pose.pose.position.y;
    pose_rpy.data[2] = pose.pose.position.z;
    pose_rpy_last.data[0] = g_pose_last.pose.position.x;
    pose_rpy_last.data[1] = g_pose_last.pose.position.y;
    pose_rpy_last.data[2] = g_pose_last.pose.position.z;

    for (int i = 0; i < 6; i++) {
      delta_pose.data[i] = (pose_rpy.data[i] - pose_rpy_last.data[i]) / dt;
    }
  } else {
    delta_pose.data.clear();
    delta_pose.data.resize(6, 0);
  }

  ROS_DEBUG_NAMED("stream_calc_dpose", "calc_dpose:dt: %2.3f", dt);
  ROS_DEBUG_STREAM_NAMED("stream_calc_dpose", "calc_dpose:pose_rpy: \n"
                                                  << pose_rpy);
  ROS_DEBUG_STREAM_NAMED("stream_calc_dpose", "calc_dpose:pose_rpy_last: \n"
                                                  << pose_rpy_last);
  ROS_DEBUG_STREAM_NAMED("stream_calc_dpose", "calc_dpose:delta_pose: \n"
                                                  << delta_pose);

  g_pose_last = pose;

  ROS_DEBUG_NAMED("stream_calc_dpose", "dur_calc_dpose: %e",
                  (ros::Time::now() - t_start).toSec());

  return delta_pose;
}

void cb_pose_quat(const geometry_msgs::PoseStamped::ConstPtr &msg) {

  if (g_state == STATE_IDLE) {
    g_state = STATE_POSE_FOLLOW;
    ROS_DEBUG_NAMED("state", "STATE_IDLE --> STATE_POSE_FOLLOW");
    ROS_DEBUG_NAMED("event", "Event: received msg on topic /pose");
    g_t_start = ros::Time::now();
  }

  g_t_last_cb = ros::Time::now();
  g_delta_pose = calc_dpose(*msg);
}

int main(int argc, char **argv) {

  // --- Initializations
  ros::init(argc, argv, "pose_following");

  // --- Setup node handles for
  //     - pose_following callbacks
  //     - start and stop service callbacks
  ros::NodeHandle nh_pose_following;
  ros::NodeHandle nh_startstop;
  ros::NodeHandle nh_pose_quat;

  // --- Setup custom callback queues for
  //     - start_pose_following and stop_pose_following callbacks
  //     - pose callback
  ros::CallbackQueue queue_startstop;
  nh_startstop.setCallbackQueue(&queue_startstop);
  ros::CallbackQueue queue_pose_quat;
  nh_pose_quat.setCallbackQueue(&queue_pose_quat);

  // --- Start an AsyncSpinner with one thread for calls to services
  //     'start_pose_following' and 'stop_pose_following'.
  // TODO does this thread need to be stopped when main() is left?
  ros::AsyncSpinner spin_startstop(1, &queue_startstop);
  spin_startstop.start();
  ros::AsyncSpinner spin_pose_quat(1, &queue_pose_quat);
  spin_pose_quat.start();

  // --- Setup publishers
  ros::Publisher streaming_pub =
      nh_pose_following.advertise<trajectory_msgs::JointTrajectory>(
          "joint_command", 10);

  // --- Setup topic subscritions
  ros::Subscriber sub_dpose = nh_pose_following.subscribe(
      "pose_following/cmd_vel", 1, cb_delta_pose_rpy);
  ros::Subscriber sub_pose =
      nh_pose_quat.subscribe("/omni1_pose", 1, cb_pose_quat);
  ros::Subscriber sub_joint =
      nh_pose_following.subscribe("joint_states", 1, cb_joint_state);

  // --- Advertise services
  //     - start: Changes state to STATE_POSE_FOLLOW
  //     - stop: Changes state to STATE_IDLE
  //     - set_velocity: Sets the jogging velocity factor
  ros::ServiceServer srv_start = nh_startstop.advertiseService(
      "/pose_following/start", cb_start_pose_following);
  ros::ServiceServer srv_stop = nh_pose_following.advertiseService(
      "/pose_following/stop", cb_stop_pose_following);

  // --- Obtain parameters
  int rate_hz = 100;
  nh_pose_following.getParam("pose_following/rate", rate_hz);
  ros::Rate loop_rate(rate_hz);

  std::string group_st = "kuka_full";
  nh_pose_following.getParam("pose_following/group", group_st);

  double w0 = 0.1;
  nh_pose_following.getParam("pose_following/w0", w0);
  double k0 = 0.001;
  nh_pose_following.getParam("pose_following/k0", k0);

  double theta_d_limit = 3.14;
  nh_pose_following.getParam("pose_following/theta_d_lim", theta_d_limit);

  double dt_pose_lim = 0.5;
  nh_pose_following.getParam("pose_following/dt_pose_lim", dt_pose_lim);

  // --- Setup MoveIt interface
  moveit::planning_interface::MoveGroupInterface move_group(group_st);

  Eigen::MatrixXd J(6, 7);
  Eigen::VectorXd theta_d(7), cart_vel(6);
  Eigen::Vector3d ref_point(0.0, 0.0, 0.0);

  cart_vel.setZero();
  theta_d.setZero();
  double dt = 0.1;
  double J_cond = 1;
  double J_cond_nakamura = 1;
  double w = 1; // Determinat-based jacobian condition
  g_delta_pose.data.resize(6, 0);

  // --- Get set up for kinematics:
  robot_model_loader::RobotModelLoader model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = model_loader.getModel();
  ROS_INFO("Model Frame: %s", kinematic_model->getModelFrame().c_str());

  moveit::core::RobotState kinematic_state(kinematic_model);
  kinematic_state.setToDefaultValues();
  const moveit::core::JointModelGroup *joint_model_group_p;
  joint_model_group_p = kinematic_model->getJointModelGroup(group_st);

  const std::vector<std::string> &joint_names =
      joint_model_group_p->getJointModelNames();
  std::vector<double> joint_values;
  kinematic_state.copyJointGroupPositions(joint_model_group_p, joint_values);
  ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
  ROS_DEBUG("joint_names: [%s %s %s %s %s %s %s]", joint_names[0].c_str(),
            joint_names[1].c_str(), joint_names[2].c_str(),
            joint_names[3].c_str(), joint_names[4].c_str(),
            joint_names[5].c_str(), joint_names[6].c_str());
  ROS_DEBUG("joint_values: [%1.2f %1.2f %1.2f %1.2f %1.2f %1.2f %1.2f]",
            joint_values[0], joint_values[1], joint_values[2], joint_values[3],
            joint_values[4], joint_values[5], joint_values[6]);
  Eigen::Isometry3d frame_tf = kinematic_state.getFrameTransform("link_t");
  std::ostream stream(nullptr);
  std::stringbuf str;
  stream.rdbuf(&str);
  kinematic_state.printTransform(frame_tf, stream);
  ROS_DEBUG_STREAM("transform: " << str.str());
  std::ostream stream2(nullptr);
  std::stringbuf str2;
  stream2.rdbuf(&str2);
  joint_model_group_p->printGroupInfo(stream2);
  ROS_DEBUG_STREAM("joint_model_group_info: " << str2.str());

  ROS_DEBUG_STREAM("g_current_joints: \n" << g_current_joints);
  // --- Initialize jogging start
  trajectory_msgs::JointTrajectory dummy_traj;
  dummy_traj.joint_names = g_current_joints.name;
  trajectory_msgs::JointTrajectoryPoint point;
  for (unsigned int joint = 0; joint < 7; joint++) {
    point.positions.push_back(g_current_joints.position.at(joint));
    point.velocities.push_back(0);
  }
  point.time_from_start = ros::Duration(0.0);
  dummy_traj.points.push_back(point);

  g_t_last = ros::Time::now();

  while (ros::ok()) {
    switch (g_state) {

    case STATE_IDLE: // IDLE

      // --- Keep track of the joint state (positions)
      for (unsigned int joint = 0; joint < 7; joint++) {
        point.positions.at(joint) = g_current_joints.position.at(joint);
        point.velocities.at(joint) = 0;
      }
      point.time_from_start = ros::Duration(0.0);
      break;

    case STATE_STOP: // STOP

      // --- Keep track of the joint state (positions)
      for (unsigned int joint = 0; joint < 7; joint++) {
        point.positions.at(joint) = g_current_joints.position.at(joint);
        point.velocities.at(joint) = 0;
      }
      point.time_from_start = ros::Duration(0.0);
      break;

    case STATE_POSE_FOLLOW: // POSE_FOLLOW

      double dt_cb = (ros::Time::now() - g_t_last_cb).toSec();
      if (dt_cb > dt_pose_lim) {
        ROS_DEBUG_NAMED("stream_dbg", "dt_cb: %1.3f", dt_cb);
        for (unsigned int i = 0; i < 6; i++) {
          g_delta_pose.data.at(i) = 0.0;
        }
        g_state = STATE_IDLE;
        ROS_DEBUG_NAMED("state", "STATE_POSE_FOLLOW --> STATE_IDLE");
        ROS_DEBUG_NAMED("event", "Event: dt_cb > dt_pose_lim");
      }

      for (int i = 0; i < 6; i++) {
        cart_vel[i] = g_delta_pose.data[i];
      }

      Eigen::Isometry3d frame_tf = kinematic_state.getFrameTransform("link_t");
      std::ostream stream(nullptr);
      std::stringbuf str;
      stream.rdbuf(&str);
      kinematic_state.printTransform(frame_tf, stream);
      ROS_DEBUG_STREAM_NAMED("stream_pose", "EEF-pose: " << str.str());

      // Time since last point:
      // ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
      dt = (ros::Time::now() - g_t_last).toSec();
      g_t_last = ros::Time::now();
      kinematic_state.setVariableValues(g_current_joints);

      ROS_DEBUG_STREAM_NAMED("stream_cart_vel", "cart_vel: \n" << cart_vel);

      // Get the Jacobian
      kinematic_state.getJacobian(
          joint_model_group_p,
          kinematic_state.getLinkModel(
              joint_model_group_p->getLinkModelNames().back()),
          ref_point, J);

      w = pow((J * J.transpose()).determinant(), 0.5);
      Eigen::MatrixXd sr_inv_J = sr_inverse(J, w, w0, k0);
      theta_d = sr_inv_J * cart_vel;
      ROS_DEBUG_STREAM_ONCE("J: \n" << J);
      ROS_DEBUG_STREAM_NAMED("stream_theta_d", "theta_d: \n" << theta_d);

      // check condition number:
      J_cond = J.norm() * p_inverse(J).norm();
      J_cond_nakamura = J.norm() * sr_inv_J.norm();
      if (w <= w0) {
        ROS_WARN_NAMED("stream_J_cnd", "Close to singularity (w = %1.6f).", w);
      }
      ROS_DEBUG_NAMED("stream_J_cnd", "Jacobian condition: %e", J_cond);
      ROS_DEBUG_NAMED("stream_J_cnd", "Jacobian condition: %e",
                      J_cond_nakamura);

      for (unsigned int j = 0; j < 7; j++) {
        if (fabs(theta_d[j]) > theta_d_limit) {
          ROS_WARN("Angular velocity of joint %d exceeding %2.2f rad/s.", j,
                   theta_d_limit);
          ROS_WARN("Changing state to STATE_STOP.");
          g_state = STATE_STOP;
          ROS_DEBUG_NAMED("state", "STATE_POSE_FOLLOW --> STATE_STOP");
          ROS_DEBUG_NAMED("event", "Event: joint velocity exceeding limit");
        }
        point.positions.at(j) = point.positions.at(j) + theta_d[j] * dt;

        point.velocities.at(j) = theta_d[j];
      }

      point.time_from_start = ros::Time::now() - g_t_start;
      dummy_traj.points.at(0) = point;
      dummy_traj.header.stamp = ros::Time::now();
      streaming_pub.publish(dummy_traj);

    } // end switch(g_state)

    loop_rate.sleep();
    // FIXME This is a blocking call that should be avoided in the future
    ros::spinOnce();

  } // end while(ros::ok())
  return 1;
}
