/*
* Copyright (C) 2015, 2016 Jannik Buckelo <jannikbu@cs.uni-bremen.de>,
* Georg Bartels <georg.bartels@cs.uni-bremen.de>
*
*
* This file is part of giskard_examples.
*
* giskard_examples is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License * along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <yaml-cpp/yaml.h>
#include <giskard/giskard.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <boost/lexical_cast.hpp>

int nWSR_;
giskard::QPController controller_;
std::vector<std::string> joint_names_;
std::vector<ros::Publisher> vel_controllers_;
geometry_msgs::PoseStamped goal_;
ros::Subscriber js_sub_;
Eigen::VectorXd state_;
bool controller_started_;
std::string frame_id_;

void js_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  if (!controller_started_)
    return;

  // TODO: turn this into a map!
  // is there a more efficient way?
  for (unsigned int i=0; i < joint_names_.size(); i++)
  {
    for (unsigned int j=0; j < msg->name.size(); j++)
    {
      if (msg->name[j].compare(joint_names_[i]) == 0)
      {
        state_[i] = msg->position[j];
      }
    }
  }

  if (controller_.update(state_, nWSR_))
  {
    Eigen::VectorXd commands = controller_.get_command();
    for (unsigned int i=0; i < vel_controllers_.size(); i++)
    {
      std_msgs::Float64 command;
      command.data = commands[i];
      vel_controllers_[i].publish(command);
    }
  }
  else
  {
    ROS_WARN("Update failed.");
    // TODO: remove or change to ros_debug
    std::cout << "State " << state_ << std::endl;
  }
}

void printGoal(const geometry_msgs::PoseStamped& goal)
{
  ROS_INFO("New goal: frame_id=%s\nposition=(%f, %f, %f), orientation=(%f, %f, %f, %f)", 
      goal.header.frame_id.c_str(), 
      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
      goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z,
      goal.pose.orientation.w);
}

void print_command(const Eigen::VectorXd& command)
{
  std::string cmd_str = " ";
  for(size_t i=0; i<command.rows(); ++i)
    cmd_str += boost::lexical_cast<std::string>(command[i]) + " ";
  ROS_INFO("Command: (%s)", cmd_str.c_str());

  using namespace KDL;
  Rotation r(Rotation::EulerZYX(command[3], command[4], command[5]));
  double x, y, z, w;
  r.GetQuaternion(x, y, z, w);
  ROS_INFO_STREAM("Command back through KDL: \nposition=(" << command[0] << 
     ", " << command[1] << ", " << command[2] << "), orientation=(" <<
    x << ", " << y << ", " << z << ", " << w << ")"); 
}

void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  printGoal(*msg);

  if(msg->header.frame_id.compare(frame_id_) != 0)
  {
    ROS_WARN("frame_id of goal did not match expected frame_id '%s'. Ignoring goal", frame_id_.c_str());
    return;
  }

  state_[joint_names_.size()] = msg->pose.position.x;
  state_[joint_names_.size() + 1] = msg->pose.position.y;
  state_[joint_names_.size() + 2] = msg->pose.position.z;

  KDL::Rotation rot;
  tf::quaternionMsgToKDL(msg->pose.orientation, rot);
  rot.GetEulerZYX(state_[joint_names_.size() + 3], state_[joint_names_.size() + 4], 
      state_[joint_names_.size() + 5]);

  print_command(state_.segment<6>(joint_names_.size()));

  if (!controller_started_)
  {
    if (controller_.start(state_, nWSR_))
    {
      ROS_INFO("Controller started.");
      goal_ = *msg;
      controller_started_ = true;
    }
    else
    {
      ROS_ERROR("Couldn't start controller.");
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pr2_controller");
  ros::NodeHandle nh("~");

  nh.param("nWSR", nWSR_, 10);

  std::string controller_description;
  if (!nh.getParam("controller_description", controller_description))
  {
    ROS_ERROR("Parameter 'controller_description' not found in namespace '%s'.", nh.getNamespace().c_str());
    return 0;
  }

  if (!nh.getParam("joint_names", joint_names_))
  {
    ROS_ERROR("Parameter 'joint_names' not found in namespace '%s'.", nh.getNamespace().c_str());
    return 0;
  }

  if (!nh.getParam("frame_id", frame_id_))
  {
    ROS_ERROR("Parameter 'frame_id' not found in namespace '%s'.", nh.getNamespace().c_str());
    return 0;
  }

  YAML::Node node = YAML::Load(controller_description);
  giskard::QPControllerSpec spec = node.as< giskard::QPControllerSpec >();
  controller_ = giskard::generate(spec);
  state_ = Eigen::VectorXd::Zero(joint_names_.size() + 6);
  controller_started_ = false;

  for (std::vector<std::string>::iterator it = joint_names_.begin(); it != joint_names_.end(); ++it)
    vel_controllers_.push_back(nh.advertise<std_msgs::Float64>("/" + *it + "/vel_cmd", 1));

  ROS_INFO("Waiting for goal.");
  ros::Subscriber goal_sub = nh.subscribe("goal", 0, goal_callback);
  js_sub_ = nh.subscribe("joint_states", 0, js_callback);
  ros::spin();

  return 0;
}
