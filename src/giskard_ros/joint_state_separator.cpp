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

#include <exception>
#include <ros/ros.h>
#include <boost/lexical_cast.hpp>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <giskard_ros/ros_utils.hpp>

namespace giskard_ros
{
  class JointStateSeparator
  {
    public:
      JointStateSeparator(const ros::NodeHandle& nh) : nh_(nh) {}
      ~JointStateSeparator() {}

      void start() 
      {
        joint_names_ = readParam< std::vector<std::string> >(nh_, "joint_names");

        for (size_t i=0; i<joint_names_.size(); ++i)
          pubs_.push_back(nh_.advertise<std_msgs::Float64>("/" + joint_names_[i].substr(0, joint_names_[i].size() - 6) + "_velocity_controller/command", 1));

        sub_ = nh_.subscribe("joint_states", 1, &JointStateSeparator::callback, this);
      }

    private:
      ros::NodeHandle nh_;
      ros::Subscriber sub_;
      std::vector<ros::Publisher> pubs_;
      std::vector<std::string> joint_names_;

      void callback(const sensor_msgs::JointState::ConstPtr& msg)
      {
        if (msg->name.size() != pubs_.size())
          throw std::runtime_error("Received message with " + boost::lexical_cast<std::string>(msg->name.size()) + " elements but excepted " + boost::lexical_cast<std::string>(pubs_.size()) + " entries.");

        if (msg->velocity.size() != msg->name.size())
          throw std::runtime_error("Received message with " + boost::lexical_cast<std::string>(msg->name.size()) + " names but " + boost::lexical_cast<std::string>(msg->velocity.size()) + " velocities.");

        std::map<std::string, double> name_cmd_map;
        for (size_t i=0; i<msg->name.size(); ++i)
          name_cmd_map.insert( std::pair<std::string, double>(msg->name[i], msg->velocity[i]) );

        for (size_t i=0; i<joint_names_.size(); ++i)
        {
          if (name_cmd_map.find(joint_names_[i]) == name_cmd_map.end())
            throw std::runtime_error("Could not find velocity command for joint '" + joint_names_[i] + "'.");

          std_msgs::Float64 out_msg;
          out_msg.data = name_cmd_map.find(joint_names_[i])->second;
          pubs_[i].publish(out_msg);
        }
      }
  };
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_state_separator");
  ros::NodeHandle nh("~");

  giskard_ros::JointStateSeparator separator(nh);

  try
  {
    separator.start();
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
  }

  return 0;
}
