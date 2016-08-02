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

#include <exception>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <giskard_msgs/WholeBodyCommand.h>
#include <kdl_conversions/kdl_msg.h>

std::vector<visualization_msgs::Marker> to_markers(const geometry_msgs::PoseStamped& msg, size_t id, double marker_length, const std::string& ns)
{
  std::vector<visualization_msgs::Marker> result;

  visualization_msgs::Marker marker;
  marker.header = msg.header;
  marker.ns = ns;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = msg.pose;
  marker.scale.x = 0.01;
  marker.color.a = 1.0;
  geometry_msgs::Point start, end;
  start.x = 0;
  start.y = 0;
  start.z = 0;


  // marker X
  marker.id = id;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

  end.x = marker_length;
  end.y = 0;
  end.z = 0;

  marker.points.push_back(start);
  marker.points.push_back(end);
  result.push_back(marker);

  // marker Y
  marker.id = id + 1;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  end.x = 0;
  end.y = marker_length;
  end.z = 0;

  marker.points.clear();
  marker.points.push_back(start);
  marker.points.push_back(end);

  result.push_back(marker);

  // marker Z
  marker.id = id + 2;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;

  end.x = 0;
  end.y = 0;
  end.z = marker_length;

  marker.points.clear();
  marker.points.push_back(start);
  marker.points.push_back(end);

  result.push_back(marker);

  return result;
}

std::vector<visualization_msgs::Marker> to_markers(const giskard_msgs::WholeBodyCommand& msg, double marker_length, const std::string& ns)
{

  std::vector<visualization_msgs::Marker> l_arm_markers, r_arm_markers, result;
  l_arm_markers = to_markers(msg.left_ee.goal, 0, marker_length, ns);
  r_arm_markers = to_markers(msg.right_ee.goal, 3, marker_length, ns);
  result.reserve(l_arm_markers.size() + r_arm_markers.size());
  result.insert(result.end(), l_arm_markers.begin(), l_arm_markers.end());
  result.insert(result.end(), r_arm_markers.begin(), r_arm_markers.end());
  return result;
}

namespace giskard_examples
{
  class GoalMarkerViz
  {
    public:
      GoalMarkerViz(const ros::NodeHandle& nh) : nh_(nh) {}
      ~GoalMarkerViz() {}
      void start() 
      {
        pub_ = nh_.advertise<visualization_msgs::MarkerArray>("markers", 1);
        sub_ = nh_.subscribe("goal", 1, &GoalMarkerViz::callback, this);
      }

    private:
      ros::NodeHandle nh_;
      ros::Subscriber sub_;
      ros::Publisher pub_;

      void callback(const giskard_msgs::WholeBodyCommand::ConstPtr& msg)
      {
        visualization_msgs::MarkerArray out_msg;
        // TODO: get this from the server or a callback?
        out_msg.markers = to_markers(*msg, 0.1, "command");
        ROS_INFO("Published marker.");
        pub_.publish(out_msg);
      }
  };
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "goal_marker_viz");
  ros::NodeHandle nh("~");

  giskard_examples::GoalMarkerViz viz(nh);

  viz.start();

  ros::spin();

  return 0;
}
