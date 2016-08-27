/*
* Copyright (C) 2016 Georg Bartels <georg.bartels@cs.uni-bremen.de>
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
* You should have received a copy of the GNU General Public License 
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#include <giskard/giskard.hpp>
#include <giskard_examples/ros_utils.hpp>
#include <ros/ros.h>

namespace giskard_examples
{
  class ControllerYamlChecker
  {
    public:
      ControllerYamlChecker(const ros::NodeHandle& nh) : nh_(nh) {}
      ~ControllerYamlChecker() {}

      void run()
      {
        std::string filename = readParam<std::string>(nh_, "filename");
        std::string file = read_ros_file(filename, "giskard_examples");
        YAML::Node node = YAML::Load(file);
        giskard::QPControllerSpec spec = node.as<giskard::QPControllerSpec>();
        giskard::QPController controller = giskard::generate(spec);
      }

    private:
      ros::NodeHandle nh_;
  };
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "check_controller_yaml");
  ros::NodeHandle nh("~");

  giskard_examples::ControllerYamlChecker checker(nh);

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
