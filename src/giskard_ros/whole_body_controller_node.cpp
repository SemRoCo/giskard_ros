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

#include <giskard_ros/whole_body_controller.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "whole_body_controller");
  ros::NodeHandle nh("~");

  giskard::ros::WholeBodyController wbc(nh);
  try
  {
    wbc.start();
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
  }

  return 0;
}
