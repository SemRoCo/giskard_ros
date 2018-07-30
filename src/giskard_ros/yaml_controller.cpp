/*
* Copyright (C) 2015-2017 Jannik Buckelo <jannikbu@cs.uni-bremen.de>,
*                         Georg Bartels <georg.bartels@cs.uni-bremen.de>
*
*
* This file is part of giskard.
*
* giskard is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2 
* of the License, or (at your option) any later version.  
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License 
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <yaml-cpp/yaml.h>
#include <giskard_core/giskard_core.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <boost/lexical_cast.hpp>

int nWSR_;
giskard_core::QPController controller_;
std::vector<std::string> joint_names_;
std::vector<ros::Publisher> vel_controllers_;
ros::Subscriber js_sub_;
ros::Publisher cmd_pub_;
Eigen::VectorXd state_;
bool controller_started_;
std_msgs::Float64MultiArray cmd_msg_;

void print_eigen(const Eigen::VectorXd& command)
{
  std::string cmd_str = " ";
  for(size_t i=0; i<command.rows(); ++i)
    cmd_str += boost::lexical_cast<std::string>(command[i]) + " ";
  ROS_DEBUG("Command: (%s)", cmd_str.c_str());
}

void js_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
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

  if (!controller_started_)
    return;

  if (controller_.update(state_, nWSR_))
  {
    Eigen::VectorXd commands = controller_.get_command();
    for (unsigned int i=0; i < vel_controllers_.size(); i++)
    {
      std_msgs::Float64 command;
      command.data = commands[i];
      cmd_msg_.data[i] = commands[i];
      vel_controllers_[i].publish(command);
    }
    cmd_pub_.publish(cmd_msg_);
  }
  else
  {
    ROS_ERROR("Update failed. Stopping controller.");
    controller_started_ = false;
    print_eigen(state_);
  }
}

void goal_callback(const std_msgs::String::ConstPtr& msg)
{
  YAML::Node node = YAML::Load(msg->data);
  giskard_core::QPControllerSpec spec = node.as< giskard_core::QPControllerSpec >();
  controller_ = giskard_core::generate(spec);
  controller_started_ = false;

  if (controller_.start(state_, nWSR_))
  {
    ROS_INFO("Controller started.");
    controller_started_ = true;
  }
  else
  {
    ROS_ERROR("Couldn't start controller. Ignoring goal.");
    print_eigen(state_);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "single_pose_controller");
  ros::NodeHandle nh("~");

  nh.param("nWSR", nWSR_, 20);

  std::string controller_description;
  if (!nh.getParam("joint_names", joint_names_))
  {
    ROS_ERROR("Parameter 'joint_names' not found in namespace '%s'.", nh.getNamespace().c_str());
    return 0;
  }

  for (std::vector<std::string>::iterator it = joint_names_.begin(); it != joint_names_.end(); ++it)
    vel_controllers_.push_back(nh.advertise<std_msgs::Float64>("/" + it->substr(0, it->size() - 6) + "_velocity_controller/command", 1));

  cmd_pub_ = nh.advertise<std_msgs::Float64MultiArray>("cmd", 1);
  cmd_msg_.data.resize(joint_names_.size());

  state_ = Eigen::VectorXd::Zero(joint_names_.size());

  ROS_INFO("Waiting for goal.");
  ros::Subscriber goal_sub = nh.subscribe("goal", 0, goal_callback);
  js_sub_ = nh.subscribe("joint_states", 0, js_callback);
  ros::spin();

  return 0;
}
