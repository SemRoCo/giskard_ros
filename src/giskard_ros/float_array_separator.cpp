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
#include <giskard_msgs/SemanticFloat64Array.h>
#include <std_msgs/Float64.h>
#include <giskard_ros/ros_utils.hpp>

namespace giskard_ros
{
  class FloatArraySeparator
  {
    public:
      FloatArraySeparator(const ros::NodeHandle& nh) : nh_(nh) {}
      ~FloatArraySeparator() {}

      void start() 
      {
        std::vector<std::string> joint_names = readParam< std::vector<std::string> >(nh_, "joint_names");

        for (size_t i=0; i<joint_names.size(); ++i)
          pubs_.push_back(nh_.advertise<std_msgs::Float64>("/" + joint_names[i].substr(0, joint_names[i].size() - 6) + "_velocity_controller/command", 1));

        sub_ = nh_.subscribe("input_array", 1, &FloatArraySeparator::callback, this);
      }

    private:
      ros::NodeHandle nh_;
      ros::Subscriber sub_;
      std::vector<ros::Publisher> pubs_;

      void callback(const giskard_msgs::SemanticFloat64Array::ConstPtr& msg)
      {
        if (msg->data.size() != pubs_.size() )
          throw std::runtime_error("Received message with " + boost::lexical_cast<std::string>(msg->data.size()) + " elements but excepted " + boost::lexical_cast<std::string>(pubs_.size()) + " entries.");

        for (size_t i=0; i<pubs_.size(); ++i)
        {
          std_msgs::Float64 out_msg;
          out_msg.data = msg->data[i].value;
          pubs_[i].publish(out_msg);
        }
      }
  };
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "float_array_separator");
  ros::NodeHandle nh("~");

  giskard_ros::FloatArraySeparator separator(nh);

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
