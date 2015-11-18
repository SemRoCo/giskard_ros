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
#include <yaml-cpp/yaml.h>
#include <giskard/giskard.hpp>

int nWSR_;
giskard::QPController controller_;
std::vector<std::string> joint_names_;
std::vector<ros::Publisher> vel_controllers_;

void js_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  Eigen::VectorXd state(joint_names_.size());
  // is there a more efficient way?
  for (unsigned int i=0; i < joint_names_.size(); i++)
  {
    for (unsigned int j=0; j < msg->name.size(); j++)
    {
      if (msg->name[j].compare(joint_names_[i]) == 0)
      {
        state[i] = msg->position[j];
      }
    }
  }

  if (controller_.update(state, nWSR_))
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
    std::cout << "State " << state << std::endl;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pr2_controller");
  ros::NodeHandle n;

  std::string pkg_path = ros::package::getPath("giskard_examples");
  YAML::Node node = YAML::LoadFile(pkg_path + "/controller_specs/pr2_qp_position_control_without_torso.yaml");

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
  Eigen::VectorXd state(joint_names_.size());

  if (controller_.start(state, nWSR_))
  {
    ROS_INFO("Controller started.");
    ros::Subscriber sub = n.subscribe("joint_states", 0, js_callback);
    ros::spin();
  }
  else
  {
    ROS_ERROR("Couldn't start controller.");
  }

  return 0;
}
