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

#include <ros/ros.h>
#include <exception>

namespace giskard_examples
{
  class PR2ControllerActionServer
  {
    public:
      PR2ControllerActionServer(const ros::NodeHandle& nh) : nh_(nh) {}
      ~PR2ControllerActionServer() {}
      void start()
      {
        // TODO: implement me
      }
    private:
      ros::NodeHandle nh_;
  };
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pr2_controller_action_server");
  ros::NodeHandle nh("~");

  giskard_examples::PR2ControllerActionServer server(nh);
  try
  {
    server.start();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
    return 0;
  }

  ros::spin();

  return 0;
}
