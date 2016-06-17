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
#include <memory>
#include <ros/ros.h>
#include <tf2_ros/buffer_client.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <giskard_examples/utils.hpp>
#include <actionlib/server/simple_action_server.h>
#include <giskard_msgs/WholeBodyAction.h>
#include <giskard_msgs/ControllerFeedback.h>

namespace giskard_examples
{
  inline giskard_msgs::WholeBodyFeedback calculateFeedback(const giskard_msgs::ControllerFeedback& msg)
  {
    // TODO: implement me
    return giskard_msgs::WholeBodyFeedback();
  }

  inline bool motionFinished(const giskard_msgs::WholeBodyFeedback& msg)
  {
    // TODO: implement me
    return false;
  }

  inline giskard_msgs::WholeBodyResult calculateResult(const giskard_msgs::WholeBodyFeedback& msg)
  {
    giskard_msgs::WholeBodyResult result;
    result.state = msg.state;
    return result;
  }

  class ControllerActionServer
  {
    public:
      ControllerActionServer(const ros::NodeHandle& nh, const std::string& name, double period,
          const std::string& tf_ns, const std::string& frame_id) : 
        nh_(nh), server_(nh, name, boost::bind(&ControllerActionServer::execute, this, _1), false),
        period_(ros::Duration(period)), frame_id_(frame_id), tf_(std::make_shared<tf2_ros::BufferClient>(tf_ns))
      {}

      ~ControllerActionServer() {}

      void start(const ros::Duration& timeout)
      {
        if(!tf_->waitForServer(timeout))
          throw std::runtime_error("Wait for TF2 BufferServer timed out. Aborting.");
        command_pub_ = nh_.advertise<giskard_msgs::WholeBodyCommand>("command", 1);
        feedback_sub_ = nh_.subscribe("feedback", 1, &ControllerActionServer::feedback, this);
        running_goal_sub_ = nh_.subscribe("running_goal", 1, &ControllerActionServer::runningGoal, this);
        server_.start();
      }

    private:
      ros::NodeHandle nh_;
      ros::Subscriber feedback_sub_, running_goal_sub_;
      ros::Publisher command_pub_;
      actionlib::SimpleActionServer<giskard_msgs::WholeBodyAction> server_;
      std::shared_ptr<tf2_ros::BufferClient> tf_;
      giskard_msgs::ControllerFeedback controller_feedback_;
      giskard_msgs::WholeBodyFeedback feedback_;
      size_t running_goal_hash_;
      ros::Duration period_;
      std::string frame_id_;

      void feedback(const giskard_msgs::ControllerFeedback::ConstPtr& msg)
      {
        controller_feedback_ = *msg;
      }

      void runningGoal(const giskard_msgs::WholeBodyCommand::ConstPtr& msg)
      {
        running_goal_hash_ = calculateHash<giskard_msgs::WholeBodyCommand>(*msg);
      }

      void execute(const giskard_msgs::WholeBodyGoalConstPtr& goal)
      {
        ROS_DEBUG("Received a new motion goal.");

        giskard_msgs::WholeBodyCommand command_msg;
        try
        {
          command_msg = transformCommand(goal->command);
        }
        catch(tf2::TransformException& ex)
        {
          ROS_ERROR("Could not transform goal. Error: %s", ex.what());
          server_.setAborted(giskard_msgs::WholeBodyResult(), ex.what());
          return;
        }
 
        do
        {
          if(server_.isPreemptRequested() || !ros::ok())
          {
            server_.setPreempted();
            return;
          }
          else
          {
            command_pub_.publish(command_msg);
            feedback_ = calculateFeedback(controller_feedback_);
            server_.publishFeedback(feedback_);
            period_.sleep();
          }
        }
        while(!motionFinished(feedback_));
        server_.setSucceeded(calculateResult(feedback_));
      }

      giskard_msgs::WholeBodyCommand transformCommand(const giskard_msgs::WholeBodyCommand& in_msg)
      {
        giskard_msgs::WholeBodyCommand out_msg;
        tf_->transform(in_msg.left_ee_goal, out_msg.left_ee_goal, frame_id_);
        tf_->transform(in_msg.right_ee_goal, out_msg.right_ee_goal, frame_id_);
        return out_msg;
      }
  };
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller_action_server");
  ros::NodeHandle nh("~");

  // TODO: get this info from the parameter server
  giskard_examples::ControllerActionServer server(nh, "move", 0.05, "/tf2_buffer_server", "base_link");
  try
  {
    server.start(ros::Duration(2.0));
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
    return 0;
  }

  ros::spin();

  return 0;
}
