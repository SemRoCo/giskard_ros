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


#ifndef __GISKARD_ROS_UTILS_HPP__
#define __GISKARD_ROS_UTILS_HPP__

#include <exception>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <giskard_msgs/SemanticFloat64.h>

namespace giskard_examples
{
  template<class T>
  inline T readParam(const ros::NodeHandle& nh, const std::string& name)
  {
    T param;
    if(!nh.getParam(name, param))
      throw std::runtime_error("Could not find parameter '" + name +
          "' in namespace '" + nh.getNamespace() + "'.");
    return param;
  }

  inline std::vector<giskard_msgs::SemanticFloat64> to_msg(
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
  
  inline geometry_msgs::Pose make_pose(const std::vector<double>& values)
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
}

#endif // __GISKARD_ROS_UTILS__HPP
