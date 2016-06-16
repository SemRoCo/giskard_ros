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

  // ECHO OF A MEANINGFUL GOAL
  //
  // left_ee_goal: 
  //  header: 
  //    seq: 0
  //    stamp: 
  //      secs: 1466101028
  //      nsecs: 986925737
  //    frame_id: base_link
  //  pose: 
  //    position: 
  //      x: 0.408104267942
  //      y: 0.0457644589442
  //      z: 0.753605697287
  //    orientation: 
  //      x: 0.461663572793
  //      y: -0.39932199905
  //      z: -0.488951012493
  //      w: 0.623165783731
  //right_ee_goal: 
  //  header: 
  //    seq: 0
  //    stamp: 
  //      secs: 1466101044
  //      nsecs: 386898175
  //    frame_id: base_link
  //  pose: 
  //    position: 
  //      x: 0.381812181883
  //      y: -0.110539927419
  //      z: 1.10948410003
  //    orientation: 
  //      x: -0.193389274
  //      y: 0.00863179543304
  //      z: 0.71675563293
  //      w: 0.669915997326
 
  if (client.waitForResult(ros::Duration(8)))
    ROS_INFO("Action finished: %s", client.getState().toString().c_str());
  else
    ROS_INFO("Action timeod out.");
  
  return 0;
}
