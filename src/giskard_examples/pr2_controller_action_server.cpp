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
      ControllerActionServer(const ros::NodeHandle& nh, const std::string& name) : 
        nh_(nh), server_(nh, name, boost::bind(&ControllerActionServer::execute, this, _1), false) 
      {}

      ~ControllerActionServer() {}

      void start()
      {
        // TODO: get this from parameter server
        period_ = ros::Duration(0.02);
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
      giskard_msgs::ControllerFeedback controller_feedback_;
      giskard_msgs::WholeBodyFeedback feedback_;
      size_t running_goal_hash_;
      ros::Duration period_;

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
        do
        {
          if(server_.isPreemptRequested() || !ros::ok())
          {
            server_.setPreempted();
            return;
          }
          else
          {
            command_pub_.publish(goal->command);
            feedback_ = calculateFeedback(controller_feedback_);
            server_.publishFeedback(feedback_);
            period_.sleep();
          }
        }
        while(!motionFinished(feedback_));
        server_.setSucceeded(calculateResult(feedback_));
      }
  };
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller_action_server");
  ros::NodeHandle nh("~");

  giskard_examples::ControllerActionServer server(nh, "move");
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
