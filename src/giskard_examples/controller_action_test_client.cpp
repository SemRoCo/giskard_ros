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

geometry_msgs::Pose make_pose(const std::vector<double>& values)
{
  geometry_msgs::Pose msg;
  msg.position.x = values[0];
  msg.position.y = values[1];
  msg.position.z = values[2];
  msg.orientation.x = values[3];
  msg.orientation.y = values[4];
  msg.orientation.z = values[5];
  msg.orientation.w = values[6];
  return msg;
}

giskard_msgs::ArmCommand make_cartesian_command(const std::vector<double>& pose,
    const std::map< std::string, double>& thresholds)
{
  giskard_msgs::ArmCommand msg;
  msg.goal_pose.header.stamp = ros::Time::now();
  msg.goal_pose.header.frame_id = "base_link";
  msg.goal_pose.pose = make_pose(pose);
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

giskard_msgs::WholeBodyGoal all_joint_goal(const ros::NodeHandle& nh)
{
  giskard_msgs::WholeBodyGoal goal;
  goal.command.left_ee = make_joint_command(
    giskard_examples::readParam< std::vector<double> >(nh, "goals/left_arm/joint"),
    giskard_examples::readParam< std::map<std::string, double> >(nh, "thresholds/left_arm/joint"));
  goal.command.right_ee = make_joint_command(
    giskard_examples::readParam< std::vector<double> >(nh, "goals/right_arm/joint"),
    giskard_examples::readParam< std::map<std::string, double> >(nh, "thresholds/right_arm/joint"));

  return goal;
}

giskard_msgs::WholeBodyGoal left_pose_goal(const ros::NodeHandle& nh)
{
  giskard_msgs::WholeBodyGoal goal;
  goal.command.left_ee = make_cartesian_command(
      giskard_examples::readParam< std::vector<double> >(nh, "goals/left_arm/cartesian"),
      giskard_examples::readParam< std::map<std::string, double> >(nh,
        "thresholds/left_arm/cartesian"));
  return goal;
}

giskard_msgs::WholeBodyGoal right_pose_goal(const ros::NodeHandle& nh)
{
  giskard_msgs::WholeBodyGoal goal;
  goal.command.right_ee = make_cartesian_command(
    giskard_examples::readParam< std::vector<double> >(nh, "goals/right_arm/cartesian"),
    giskard_examples::readParam< std::map<std::string, double> >(nh,
      "thresholds/right_arm/cartesian"));
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
