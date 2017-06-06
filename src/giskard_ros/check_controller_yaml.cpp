/*
* Copyright (C) 2016-2017 Georg Bartels <georg.bartels@cs.uni-bremen.de>
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

#include <giskard_core/giskard_core.hpp>
#include <giskard_ros/ros_utils.hpp>
#include <ros/ros.h>

namespace giskard_ros
{
  class ControllerYamlChecker
  {
    public:
      ControllerYamlChecker(const ros::NodeHandle& nh) : nh_(nh) {}
      ~ControllerYamlChecker() {}

      void run()
      {
        std::string filename = readParam<std::string>(nh_, "filename");
        std::string file = read_ros_file(filename, "giskard_ros");
        YAML::Node node = YAML::Load(file);
        giskard_core::QPControllerSpec spec = node.as<giskard_core::QPControllerSpec>();
        giskard_core::QPController controller = giskard_core::generate(spec);
      }

    private:
      ros::NodeHandle nh_;
  };
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "check_controller_yaml");
  ros::NodeHandle nh("~");

  giskard_ros::ControllerYamlChecker checker(nh);

  try
  {
    checker.run();
    ROS_INFO("Successfully parsed and generated controller.");
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
  }

  return 0;
}
