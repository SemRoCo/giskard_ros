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
#include <sensor_msgs/JointState.h>
#include <giskard_examples/ros_utils.hpp>

namespace giskard_examples
{
  class JointStatePrettyPrinter
  {
    public:
      JointStatePrettyPrinter(const ros::NodeHandle& nh) : nh_(nh) {}
      ~JointStatePrettyPrinter() {}

      void start()
      {
        sub_ = nh_.subscribe("/joint_states", 1, &JointStatePrettyPrinter::callback, this);
      }

      const ros::NodeHandle& get_node_handle() const
      {
        return nh_;
      }

    private:
      ros::NodeHandle nh_;
      ros::Subscriber sub_;

      void callback(const sensor_msgs::JointState::ConstPtr& msg)
      {
        if (msg->name.size() != msg->position.size())
          throw std::runtime_error("Size of names and positions in message of type sensor_msgs/JointState did not match.");

        std::cout << "\nJOINT STATES:\n";
        for (size_t i=0; i<msg->name.size(); ++i)
          std::cout << msg->name[i] << ": " << msg->position[i] << std::endl;

        nh_.shutdown();
      }
  };

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_state_pretty_printer");
  ros::NodeHandle nh("~");

  giskard_examples::JointStatePrettyPrinter printer(nh);

  try
  {
    printer.start();
    while (printer.get_node_handle().ok())
      ros::spinOnce();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
  }

  return 0;
}
