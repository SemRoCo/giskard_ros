/*
* Copyright (C) 2015 Jannik Buckelo <jannikbu@cs.uni-bremen.de>
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
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <yaml-cpp/yaml.h>
#include <giskard/giskard.hpp>

int nWSR_;
giskard::QPController controller_;
std::vector<std::string> joint_names_;
std::vector<ros::Publisher> vel_controllers_;
geometry_msgs::Point goal_point_;
ros::Subscriber js_sub_;
Eigen::VectorXd state_;
bool controller_started_;

void js_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  if (!controller_started_)
    return;

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

void goal_callback(const geometry_msgs::Point::ConstPtr& msg)
{
  ROS_INFO("New goal: %f, %f, %f", msg->x, msg->y, msg->z);

  state_[joint_names_.size()] = msg->x;
  state_[joint_names_.size() + 1] = msg->y;
  state_[joint_names_.size() + 2] = msg->z;

  if (!controller_started_)
  {
    if (controller_.start(state_, nWSR_))
    {
      ROS_INFO("Controller started.");
      goal_point_ = *msg;
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
  if (argc != 2)
  {
    std::cout << "Usage: rosrun giskard_examples pr2_controller <controller_specification>" << std::endl;
    return 0;
  }
  YAML::Node node = YAML::LoadFile(argv[1]);

  ros::init(argc, argv, "pr2_controller");
  ros::NodeHandle n;

  ros::param::param<int>("giskard_examples/nWSR", nWSR_, 10);

  if (!ros::param::get("giskard_examples/joint_names", joint_names_))
  {
    ROS_ERROR("Parameter 'giskard_examples/joint_names' not found.");
    return 0;
  }

  std::vector<std::string> vel_controller_names;
  if (!ros::param::get("giskard_examples/velocity_controller_names", vel_controller_names))
  {
    ROS_ERROR("Parameter 'giskard_examples/velocity_controller_names' not found.");
    return 0;
  }

  if (joint_names_.size() != vel_controller_names.size())
  {
    ROS_ERROR("Different number of joint names and controller names.");
    return 0;
  }

  for (std::vector<std::string>::iterator it = vel_controller_names.begin(); it != vel_controller_names.end(); ++it)
  {
    vel_controllers_.push_back(n.advertise<std_msgs::Float64>(*it, 10));
  }

  giskard::QPControllerSpec spec = node.as< giskard::QPControllerSpec >();
  controller_ = giskard::generate(spec);
  state_ = Eigen::VectorXd::Zero(joint_names_.size() + 3);
  controller_started_ = false;

  ROS_INFO("Waiting for goal.");
  ros::Subscriber goal_sub = n.subscribe("/pr2_controller/goal", 0, goal_callback);
  js_sub_ = n.subscribe("joint_states", 0, js_callback);
  ros::spin();

  return 0;
}
