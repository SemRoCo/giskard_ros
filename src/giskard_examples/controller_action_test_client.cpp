#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <giskard_msgs/WholeBodyAction.h>
#include <giskard_examples/ros_utils.hpp>

std::vector<giskard_msgs::SemanticFloat64> to_msg(
    const std::map<std::string, double> map)
{
  std::vector<giskard_msgs::SemanticFloat64> result;
  for (std::map<std::string, double>::const_iterator it=map.begin();
       it!=map.end(); ++it)
  {
    giskard_msgs::SemanticFloat64 msg;
    msg.semantics = it->first;
    msg.value = it->second;
    result.push_back(msg);
  }
  return result;
}

giskard_msgs::ArmCommand make_cartesian_command(const geometry_msgs::Pose& pose,
    const std::map< std::string, double>& thresholds)
{
  giskard_msgs::ArmCommand msg;
  msg.goal_pose.header.stamp = ros::Time::now();
  msg.goal_pose.header.frame_id = "base_link";
  msg.goal_pose.pose = pose;
  msg.type = giskard_msgs::ArmCommand::CARTESIAN_GOAL;
  msg.convergence_thresholds = to_msg(thresholds);
  return msg;
}

giskard_msgs::ArmCommand make_joint_command(const std::vector<double> config,
    const std::map<std::string, double> thresholds)
{
  giskard_msgs::ArmCommand msg;
  msg.goal_configuration = config;
  msg.type = giskard_msgs::ArmCommand::JOINT_GOAL;
  msg.convergence_thresholds = to_msg(thresholds);
  return msg;
}

geometry_msgs::Pose make_pose(double x, double y, double z, 
    double qx, double qy, double qz, double qw)
{
  geometry_msgs::Pose msg;
  msg.position.x = x;
  msg.position.y = y;
  msg.position.z = z;
  msg.orientation.x = qx;
  msg.orientation.y = qy;
  msg.orientation.z = qz;
  msg.orientation.w = qw;
  return msg;
}

geometry_msgs::Pose right_arm_pose()
{
  return make_pose(0.381812181883, -0.110539927419, 1.10948410003,
        -0.193389274, 0.00863179543304, 0.71675563293, 0.669915997326);
}

geometry_msgs::Pose left_arm_pose()
{
  return make_pose(0.508104267942, 0.0457644589442, 0.653605697287,
        0.461663572793, -0.39932199905, -0.488951012493, 0.623165783731);
}

std::vector<double> left_arm_joint_config()
{
  return {0.9, 0.5, 0.9, -1.6, -5.0, -1.4, -1.8};
}

std::vector<double> right_arm_joint_config()
{
  return {-0.9, 0.5, -0.9, -1.6, 5.0, -1.4, 1.8};
}

giskard_msgs::WholeBodyGoal all_joint_goal(const ros::NodeHandle& nh)
{
  giskard_msgs::WholeBodyGoal goal;
  goal.command.left_ee = make_joint_command(left_arm_joint_config(),
    giskard_examples::readParam< std::map<std::string, double> >(nh, "thresholds/left_arm/joint_control"));
  goal.command.right_ee = make_joint_command(right_arm_joint_config(),
    giskard_examples::readParam< std::map<std::string, double> >(nh, "thresholds/right_arm/joint_control"));

  return goal;
}

giskard_msgs::WholeBodyGoal left_pose_goal(const ros::NodeHandle& nh)
{
  giskard_msgs::WholeBodyGoal goal;
  goal.command.left_ee = make_cartesian_command(left_arm_pose(),
      giskard_examples::readParam< std::map<std::string, double> >(nh,
        "thresholds/left_arm/cartesian_control"));
  return goal;
}

giskard_msgs::WholeBodyGoal right_pose_goal(const ros::NodeHandle& nh)
{
  giskard_msgs::WholeBodyGoal goal;
  goal.command.right_ee = make_cartesian_command(right_arm_pose(),
      giskard_examples::readParam< std::map<std::string, double> >(nh,
        "thresholds/right_arm/cartesian_control"));
  return goal;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "controller_action_test_client");
  ros::NodeHandle nh("~");

  actionlib::SimpleActionClient<giskard_msgs::WholeBodyAction> client("/controller_action_server/move", true);
  
  ROS_INFO("Waiting for action server to start.");
  client.waitForServer();
  ROS_INFO("Action server started, sending goal.");

  client.sendGoal(all_joint_goal(nh));
  if (client.waitForResult(ros::Duration(10)))
    ROS_INFO("Action finished: %s", client.getState().toString().c_str());
  else
    ROS_INFO("Action timed out.");

  client.sendGoal(left_pose_goal(nh));
  if (client.waitForResult(ros::Duration(10)))
    ROS_INFO("Action finished: %s", client.getState().toString().c_str());
  else
    ROS_INFO("Action timed out.");

  client.sendGoal(all_joint_goal(nh));
  if (client.waitForResult(ros::Duration(10)))
    ROS_INFO("Action finished: %s", client.getState().toString().c_str());
  else
    ROS_INFO("Action timed out.");

  client.sendGoal(right_pose_goal(nh));
  if (client.waitForResult(ros::Duration(10)))
    ROS_INFO("Action finished: %s", client.getState().toString().c_str());
  else
    ROS_INFO("Action timed out.");

//  // SEND FIRST GOAL FOR ENTIRE BODY WHICH TIMES OUT AFTER 10s
//  client.sendGoal(make_first_goal());
//
//  if (client.waitForResult(ros::Duration(10)))
//    ROS_INFO("Action finished: %s", client.getState().toString().c_str());
//  else
//    ROS_INFO("Action timed out.");
//
//  // SEND SECOND GOAL FOR LEFT ARM ONLY WITHOUT WAITING FOR IT TO FINISH
//  ROS_INFO("Sending second goal.");
//  client.sendGoal(make_second_goal());
//  ros::Duration(1.0).sleep();
//
//
//  // SEND THIRD GOAL FOR LEFT ARM ONLY, SHOULD AUTOMATICALLY PREEMPT SECOND GOAL
//  ROS_INFO("Sent final goal to abort second one.");
//  client.sendGoal(make_third_goal());
//
//  if (client.waitForResult(ros::Duration(10)))
//    ROS_INFO("Action finished: %s", client.getState().toString().c_str());
//  else
//    ROS_INFO("Action timed out.");

  return 0;
}
