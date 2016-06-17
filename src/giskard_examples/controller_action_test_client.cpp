#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <giskard_msgs/WholeBodyAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "controller_action_test_client");

  actionlib::SimpleActionClient<giskard_msgs::WholeBodyAction> client("/pr2_controller_action_server/move", true);
  
  ROS_INFO("Waiting for action server to start.");
  client.waitForServer();
  ROS_INFO("Action server started, sending goal.");

  // SEND FIRST GOAL WHICH TIMES OUT AFTER 10s
  giskard_msgs::WholeBodyGoal goal;
  goal.command.left_ee_goal.header.stamp = ros::Time::now();
  goal.command.left_ee_goal.header.frame_id = "base_link";
  goal.command.left_ee_goal.pose.position.x = 0.408104267942;
  goal.command.left_ee_goal.pose.position.y = 0.0457644589442;
  goal.command.left_ee_goal.pose.position.z = 0.753605697287;
  goal.command.left_ee_goal.pose.orientation.x = 0.461663572793;
  goal.command.left_ee_goal.pose.orientation.y = -0.39932199905;
  goal.command.left_ee_goal.pose.orientation.z = -0.488951012493;
  goal.command.left_ee_goal.pose.orientation.w = 0.623165783731;

  goal.command.right_ee_goal.header.stamp = ros::Time::now();
  goal.command.right_ee_goal.header.frame_id = "base_link";
  goal.command.right_ee_goal.pose.position.x = 0.381812181883;
  goal.command.right_ee_goal.pose.position.y = -0.110539927419;
  goal.command.right_ee_goal.pose.position.z = 1.10948410003;
  goal.command.right_ee_goal.pose.orientation.x = -0.193389274;
  goal.command.right_ee_goal.pose.orientation.y = 0.00863179543304;
  goal.command.right_ee_goal.pose.orientation.z = 0.71675563293;
  goal.command.right_ee_goal.pose.orientation.w = 0.669915997326;

  client.sendGoal(goal);

  if (client.waitForResult(ros::Duration(10)))
    ROS_INFO("Action finished: %s", client.getState().toString().c_str());
  else
    ROS_INFO("Action timed out.");

  // SEND SECOND GOAL WITHOUT WAITING FOR IT TO FINISH

  goal.command.left_ee_goal.header.stamp = ros::Time::now();
  goal.command.right_ee_goal.header.stamp = ros::Time::now();
  goal.command.left_ee_goal.pose.position.z = 0.653605697287;
  goal.command.right_ee_goal.pose.position.z = 1.20948410003;

  client.sendGoal(goal);

  ros::Duration(1.0).sleep();


  // SEND THIRD GOAL, SHOULD AUTOMATICALLY PREEMPT SECOND GOAL

  goal.command.left_ee_goal.header.stamp = ros::Time::now();
  goal.command.right_ee_goal.header.stamp = ros::Time::now();
  goal.command.left_ee_goal.pose.position.x = 0.508104267942;
  goal.command.right_ee_goal.pose.position.x = 0.481812181883;

  ROS_INFO("Sent final goal to finish first one.");
  client.sendGoal(goal);

  if (client.waitForResult(ros::Duration(10)))
    ROS_INFO("Action finished: %s", client.getState().toString().c_str());
  else
    ROS_INFO("Action timed out.");


  return 0;
}
