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

  client.sendGoal(giskard_msgs::WholeBodyGoal());
  
  if (client.waitForResult(ros::Duration(1.5)))
    ROS_INFO("Action finished: %s", client.getState().toString().c_str());
  else
    ROS_INFO("Action timeod out.");
  
  return 0;
}
