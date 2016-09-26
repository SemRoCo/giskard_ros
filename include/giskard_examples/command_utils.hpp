/*
* Copyright (C) 2016 Georg Bartels <georg.bartels@cs.uni-bremen.de>
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

#ifndef __GISKARD_COMMANDS_UTILS_HPP__
#define __GISKARD_COMMANDS_UTILS_HPP__

#include <vector>
#include <string>
#include <exception>
#include <giskard_examples/conversions.hpp>
#include <giskard_examples/whole_body_controller.hpp>

namespace giskard_examples
{
  inline void sanity_check(const std::vector<double>& v, 
      const std::vector<std::string>& joint_names, const std::string& name)
  {
    if (v.size() != joint_names.size())
      throw std::runtime_error("Did not find expected number of values for " + name + " joint goal.");
  }

  inline void sanity_check(const geometry_msgs::PoseStamped& msg, 
      const std::string& frame_id, const std::string& name)
  {
    if(msg.header.frame_id.compare(frame_id) != 0)
      throw std::runtime_error("frame_id of " + name + " goal did not match '" + 
          frame_id +"'.");
  }

  inline void sanity_check(const giskard_msgs::ArmCommand& msg, 
      const std::vector<std::string>& joint_names,
      const std::string& frame_id, const std::string& name)
  {
    switch (msg.type)
    {
      case giskard_msgs::ArmCommand::JOINT_GOAL:
        sanity_check(msg.goal_configuration, joint_names, name);
        break;
      case giskard_msgs::ArmCommand::CARTESIAN_GOAL:
        sanity_check(msg.goal_pose, frame_id, name);
        break;
      default:
        throw std::runtime_error("Received command of unknown type for " + name + ".");
        break;
    }
  }

  inline void sanity_check(const giskard_msgs::WholeBodyCommand& command,
      const giskard_examples::WholeBodyControllerParams& params)
  {
    switch (command.type)
    {
      case (giskard_msgs::WholeBodyCommand::STANDARD_CONTROLLER):
        sanity_check(command.right_ee, params.r_arm_names, params.frame_id, "left arm");
        sanity_check(command.left_ee, params.l_arm_names, params.frame_id, "right arm");
        break;
      case (giskard_msgs::WholeBodyCommand::YAML_CONTROLLER):
        break;
      default:
        throw std::runtime_error("Received unknown type for whole-body controller.");
    }
  }

  inline giskard_msgs::ArmCommand init_arm_joint_command(const sensor_msgs::JointState& msg, 
      const std::vector<std::string>& joint_names)
  {
    giskard_msgs::ArmCommand result;
    result.type = giskard_msgs::ArmCommand::JOINT_GOAL;
    for (size_t i=0; i<joint_names.size(); ++i)
      for (size_t j=0; j<msg.name.size(); ++j)
        if (msg.name[j].compare(joint_names[i]) == 0)
          result.goal_configuration.push_back(msg.position[j]);

    return result;
  }
  
  inline giskard_msgs::ArmCommand init_arm_cart_command(const geometry_msgs::PoseStamped& pose)
  {
    giskard_msgs::ArmCommand result;
    result.type = giskard_msgs::ArmCommand::CARTESIAN_GOAL;
    result.goal_pose = pose;
    return result;
  }

  inline giskard_msgs::WholeBodyCommand init_joint_joint_command (
      const sensor_msgs::JointState& msg, const WholeBodyControllerParams& params)
  {
    giskard_msgs::WholeBodyCommand result;
    result.right_ee = init_arm_joint_command(msg, params.r_arm_names);
    result.left_ee = init_arm_joint_command(msg, params.l_arm_names);
    return result;
  }

  inline giskard_msgs::WholeBodyCommand init_cart_cart_command (
      const ros::Time& stamp, const std::string& frame_id,
      const KDL::Frame& l_pose, const KDL::Frame& r_pose)
  {
    giskard_msgs::WholeBodyCommand result;
    result.left_ee = init_arm_cart_command(to_msg(stamp, frame_id, l_pose));
    result.right_ee = init_arm_cart_command(to_msg(stamp, frame_id, r_pose));
    return result;
  }

  inline giskard_msgs::WholeBodyCommand init_cart_joint_command (
      const sensor_msgs::JointState& msg, const KDL::Frame& pose,
      const WholeBodyControllerParams& params)
  {
    giskard_msgs::WholeBodyCommand result;
    result.left_ee = init_arm_cart_command(
        to_msg(msg.header.stamp, params.frame_id, pose));
    result.right_ee = init_arm_joint_command(msg, params.r_arm_names);
    return result;
  }

  inline giskard_msgs::WholeBodyCommand init_joint_cart_command (
      const sensor_msgs::JointState& msg, const KDL::Frame& pose,
      const WholeBodyControllerParams& params)
  {
    giskard_msgs::WholeBodyCommand result;
    result.left_ee = init_arm_joint_command(msg, params.l_arm_names);
    result.right_ee = init_arm_cart_command(
        to_msg(msg.header.stamp, params.frame_id, pose));
    return result;
  }

}

#endif // __GISKARD_COMMANDS_UTILS__HPP
