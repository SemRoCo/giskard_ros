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
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <memory>
#include <boost/bind.hpp>

class InteractiveMarker
{
  public:
    InteractiveMarker(const ros::NodeHandle& nh, const std::string& frame_id) :
        nh_( nh ), frame_id_( frame_id)
    {}

    void start()
    {
      goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("goal_out", 1);

      server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>
          ("interactive_marker", "", false);

      visualization_msgs::InteractiveMarker marker =
          create_6dof_marker(frame_id_, 0.2);
      server_->insert(marker);
      server_->setCallback(marker.name, 
          boost::bind(&InteractiveMarker::feedback_callback, this, _1));

      server_->applyChanges();
    }
  
    ~InteractiveMarker() {}

  private:
    ros::NodeHandle nh_;
    ros::Publisher goal_pub_;
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    std::string frame_id_;

    void feedback_callback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
    {
      if (feedback->event_type ==
          visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
      {
        geometry_msgs::PoseStamped goal_msg;
        goal_msg.header = feedback->header;
        goal_msg.pose = feedback->pose;
        goal_pub_.publish(goal_msg);

        geometry_msgs::Pose identity_pose;
        identity_pose.orientation.w = 1.0;
        server_->setPose( feedback->marker_name, identity_pose); 
        server_->applyChanges();
      }

    }

    visualization_msgs::InteractiveMarker create_6dof_marker(const std::string& frame_id, double marker_scale) const
    {
      visualization_msgs::InteractiveMarker result;
      result.name = "6dof_control";
      result.description = "6-DOF EE-Control";
      result.header.frame_id = frame_id;
      result.scale = marker_scale;

      visualization_msgs::InteractiveMarkerControl control;
      control.always_visible = true;
      control.orientation.w = 1;
      control.orientation.x = 1;
      control.orientation.y = 0;
      control.orientation.z = 0;
      control.name = "rotate_x";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      result.controls.push_back(control);
      control.name = "move_x";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      result.controls.push_back(control);

      control.orientation.w = 1;
      control.orientation.x = 0;
      control.orientation.y = 1;
      control.orientation.z = 0;
      control.name = "rotate_z";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      result.controls.push_back(control);
      control.name = "move_z";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      result.controls.push_back(control);

      control.orientation.w = 1;
      control.orientation.x = 0;
      control.orientation.y = 0;
      control.orientation.z = 1;
      control.name = "rotate_y";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      result.controls.push_back(control);
      control.name = "move_y";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      result.controls.push_back(control);

      return result;
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "interactive_marker");
  ros::NodeHandle nh("~");

  std::string frame_id;
  if (!nh.getParam("frame_id", frame_id))
  {
    ROS_ERROR("Parameter 'frame_id' not found in namespace '%s'.", nh.getNamespace().c_str());
    return 0;
  }

  InteractiveMarker gim(nh, frame_id);
  gim.start();

  ros::spin();
}
