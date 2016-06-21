#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <giskard_msgs/WholeBodyAction.h>

giskard_msgs::SemanticArmCommand make_arm_command(const geometry_msgs::Pose& pose)
{
  giskard_msgs::SemanticArmCommand msg;
  msg.goal.header.stamp = ros::Time::now();
  msg.goal.header.frame_id = "base_link";
  msg.goal.pose = pose;
  msg.process = true;
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

giskard_msgs::WholeBodyGoal make_first_goal()
{
  giskard_msgs::WholeBodyGoal goal;
  goal.command.left_ee = make_arm_command(make_pose(0.408104267942, 0.0457644589442, 0.753605697287,
        0.461663572793, -0.39932199905, -0.488951012493, 0.623165783731));
  goal.command.right_ee = make_arm_command(make_pose(0.381812181883, -0.110539927419, 1.10948410003,
        -0.193389274, 0.00863179543304, 0.71675563293, 0.669915997326));
  return goal;
}

giskard_msgs::WholeBodyGoal make_second_goal()
{
  giskard_msgs::WholeBodyGoal goal;
  goal.command.left_ee = make_arm_command(make_pose(0.408104267942, 0.0457644589442, 0.653605697287,
        0.461663572793, -0.39932199905, -0.488951012493, 0.623165783731));
  return goal;
}

giskard_msgs::WholeBodyGoal make_third_goal()
{
  giskard_msgs::WholeBodyGoal goal;
  goal.command.left_ee = make_arm_command(make_pose(0.508104267942, 0.0457644589442, 0.653605697287,
        0.461663572793, -0.39932199905, -0.488951012493, 0.623165783731));
  return goal;
}
int main (int argc, char **argv)
{
  ros::init(argc, argv, "controller_action_test_client");

  actionlib::SimpleActionClient<giskard_msgs::WholeBodyAction> client("/controller_action_server/move", true);
  
  ROS_INFO("Waiting for action server to start.");
  client.waitForServer();
  ROS_INFO("Action server started, sending goal.");

  // SEND FIRST GOAL FOR ENTIRE BODY WHICH TIMES OUT AFTER 10s
  client.sendGoal(make_first_goal());

  if (client.waitForResult(ros::Duration(10)))
    ROS_INFO("Action finished: %s", client.getState().toString().c_str());
  else
    ROS_INFO("Action timed out.");

  // SEND SECOND GOAL FOR LEFT ARM ONLY WITHOUT WAITING FOR IT TO FINISH
  ROS_INFO("Sending second goal.");
  client.sendGoal(make_second_goal());
  ros::Duration(1.0).sleep();


  // SEND THIRD GOAL FOR LEFT ARM ONLY, SHOULD AUTOMATICALLY PREEMPT SECOND GOAL
  ROS_INFO("Sent final goal to abort second one.");
  client.sendGoal(make_third_goal());

  if (client.waitForResult(ros::Duration(10)))
    ROS_INFO("Action finished: %s", client.getState().toString().c_str());
  else
    ROS_INFO("Action timed out.");

  return 0;
}
