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
#include <giskard_examples/ros_utils.hpp>
#include <actionlib/server/simple_action_server.h>
#include <giskard_msgs/WholeBodyAction.h>
#include <giskard_msgs/ControllerFeedback.h>
#include <std_msgs/UInt64.h>
#include <algorithm>

namespace giskard_examples
{
  inline std::map<std::string, double> toIndex(const std::vector<giskard_msgs::SemanticFloat64>& msgs)
  {
    std::map<std::string, double> index;
    for (size_t i=0; i<msgs.size(); ++i)
      index.insert(std::pair<std::string, double>(msgs[i].semantics, msgs[i].value));

    return index;
  }

  inline double exception_lookup(const std::map<std::string, double>& index, const std::string& name)
  {
    std::map<std::string, double>::const_iterator it = index.find(name);
    if( it == index.end())
      throw std::runtime_error("Could not find element with semantics '" + name +
        "' in feedback message from controller.");

    return it->second;
  }

  inline double maxAbsValue(double a, double b)
  {
    if (std::abs(a) > std::abs(b))
      return a;
    else
      return b;
  }

  inline double lookupMaxAbsValue(const std::map<std::string, double>& index, 
    const std::vector<std::string>& search_names)
  {
    double result = 0.0;
    for (size_t i=0; i<search_names.size(); ++i)
      result = maxAbsValue(result, exception_lookup(index, search_names[i]));
    
    return result;
  }

  class WholeBodyControllables
  {
    public:
      std::vector<std::string> left_arm;
      std::vector<std::string> right_arm;
      std::string torso;
  };

  class FeedbackThresholds
  {
    public:
      ros::Duration motion_old;
      double bodypart_moves, pos_convergence, rot_convergence;
  };

  inline giskard_msgs::WholeBodyFeedback calculateFeedback(const giskard_msgs::ControllerFeedback& msg,
      const ros::Time& motion_start_time, const WholeBodyControllables& body_controllables,
      const FeedbackThresholds& thresholds, size_t running_command_hash,
      size_t current_command_hash)
  {
    giskard_msgs::WholeBodyFeedback result;
    result.state.header = msg.header;
    result.state.running_time = msg.header.stamp - motion_start_time;

    std::map<std::string, double> command_index = toIndex(msg.commands);
    std::map<std::string, double> doubles_index = toIndex(msg.doubles);

    result.state.left_arm_max_vel = 
      lookupMaxAbsValue(command_index, body_controllables.left_arm);
    result.state.right_arm_max_vel = 
      lookupMaxAbsValue(command_index, body_controllables.right_arm);
    result.state.torso_vel = 
      exception_lookup(command_index, body_controllables.torso);
    result.state.left_arm_pos_error =
      exception_lookup(doubles_index, "l_trans_error");
    result.state.left_arm_rot_error =
      exception_lookup(doubles_index, "l_rot_error");
    result.state.right_arm_pos_error =
      exception_lookup(doubles_index, "r_trans_error");
    result.state.right_arm_rot_error =
      exception_lookup(doubles_index, "r_rot_error");

    result.state.motion_started = 
      running_command_hash == current_command_hash;
    result.state.motion_old = result.state.running_time > thresholds.motion_old;
    result.state.torso_moving =
      std::abs(result.state.torso_vel) > std::abs(thresholds.bodypart_moves);
    result.state.left_arm_moving =
      std::abs(result.state.left_arm_max_vel) > std::abs(thresholds.bodypart_moves);
    result.state.right_arm_moving =
      std::abs(result.state.right_arm_max_vel) > std::abs(thresholds.bodypart_moves);
    result.state.left_arm_pos_converged =
      std::abs(result.state.left_arm_pos_error) < std::abs(thresholds.pos_convergence);
    result.state.left_arm_rot_converged =
      std::abs(result.state.left_arm_rot_error) < std::abs(thresholds.rot_convergence);
    result.state.right_arm_pos_converged =
      std::abs(result.state.right_arm_pos_error) < std::abs(thresholds.pos_convergence);
    result.state.right_arm_rot_converged =
      std::abs(result.state.right_arm_rot_error) < std::abs(thresholds.rot_convergence);

    return result;
  }

  inline bool motionFinished(const giskard_msgs::WholeBodyFeedback& msg)
  {
    return msg.state.motion_started && msg.state.motion_old && 
      !msg.state.torso_moving && !msg.state.left_arm_moving && !msg.state.right_arm_moving &&
      msg.state.left_arm_pos_converged && msg.state.left_arm_rot_converged &&
      msg.state.right_arm_pos_converged && msg.state.right_arm_rot_converged;
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
          const std::string& tf_ns, const std::string& frame_id, const WholeBodyControllables& body_controllables,
          const FeedbackThresholds& thresholds) : 
        nh_(nh), server_(nh, name, boost::bind(&ControllerActionServer::execute, this, _1), false),
        period_(ros::Duration(period)), frame_id_(frame_id), tf_(std::make_shared<tf2_ros::BufferClient>(tf_ns)),
        body_controllables_(body_controllables), thresholds_(thresholds)
      {}

      ~ControllerActionServer() {}

      void start(const ros::Duration& timeout)
      {
        if(!tf_->waitForServer(timeout))
          throw std::runtime_error("Wait for TF2 BufferServer timed out. Aborting.");

        command_pub_ = nh_.advertise<giskard_msgs::WholeBodyCommand>("command", 1);
        feedback_sub_ = nh_.subscribe("feedback", 1, &ControllerActionServer::feedback, this);
        current_command_sub_ = nh_.subscribe("current_command_hash", 1, &ControllerActionServer::currentCommandHash, this);
        server_.start();
      }

    private:
      ros::NodeHandle nh_;
      ros::Subscriber feedback_sub_, current_command_sub_;
      ros::Publisher command_pub_;
      actionlib::SimpleActionServer<giskard_msgs::WholeBodyAction> server_;
      std::shared_ptr<tf2_ros::BufferClient> tf_;
      giskard_msgs::ControllerFeedback controller_feedback_;
      giskard_msgs::WholeBodyFeedback feedback_;
      size_t running_command_hash_, current_command_hash_;
      ros::Duration period_;
      ros::Time motion_start_time_;
      std::string frame_id_;
      WholeBodyControllables body_controllables_;
      FeedbackThresholds thresholds_;

      void feedback(const giskard_msgs::ControllerFeedback::ConstPtr& msg)
      {
        controller_feedback_ = *msg;
        // TODO: refactor by getting rid of 'controller_feedback_'
        feedback_ = calculateFeedback(controller_feedback_, motion_start_time_, 
            body_controllables_, thresholds_, running_command_hash_,
            current_command_hash_);
      }

      void currentCommandHash(const std_msgs::UInt64::ConstPtr& msg)
      {
        running_command_hash_ = msg->data;
      }

      void execute(const giskard_msgs::WholeBodyGoalConstPtr& goal)
      {
        ROS_DEBUG("Received a new motion goal.");

        giskard_msgs::WholeBodyCommand current_command;
        try
        {
          current_command = processCommand(goal->command);
        }
        catch(tf2::TransformException& ex)
        {
          ROS_ERROR("Could not transform goal. Error: %s", ex.what());
          server_.setAborted(giskard_msgs::WholeBodyResult(), ex.what());
          return;
        }
 
        motion_start_time_ = ros::Time::now();

        current_command_hash_ = calculateHash<giskard_msgs::WholeBodyCommand>(current_command);

        do
        {
          if(server_.isPreemptRequested() || !ros::ok())
          {
            server_.setPreempted();
            return;
          }
          else
          {
            command_pub_.publish(current_command);
            server_.publishFeedback(feedback_);
            period_.sleep();
          }
        }
        while(!motionFinished(feedback_));
        server_.setSucceeded(calculateResult(feedback_));
      }

      giskard_msgs::WholeBodyCommand processCommand(const giskard_msgs::WholeBodyCommand& in_msg)
      {
        giskard_msgs::WholeBodyCommand out_msg = in_msg;
        if(in_msg.left_ee.process)
          tf_->transform(in_msg.left_ee.goal, out_msg.left_ee.goal, frame_id_);
        if(in_msg.right_ee.process)
          tf_->transform(in_msg.right_ee.goal, out_msg.right_ee.goal, frame_id_);

        return out_msg;
      }
  };
}

int main(int argc, char **argv)
{
  using namespace giskard_examples;
  ros::init(argc, argv, "controller_action_server");
  ros::NodeHandle nh("~");

  FeedbackThresholds thresholds;
  thresholds.motion_old = ros::Duration(readParam<double>(nh, "thresholds/motion_old"));
  thresholds.bodypart_moves = readParam<double>(nh, "thresholds/bodypart_moves");
  thresholds.pos_convergence = readParam<double>(nh, "thresholds/pos_convergence");
  thresholds.rot_convergence = readParam<double>(nh, "thresholds/rot_convergence");

  WholeBodyControllables body_controllables;
  body_controllables.left_arm = readParam< std::vector<std::string> >
    (nh, "body_controllables/left_arm");
  body_controllables.right_arm = readParam< std::vector<std::string> >
    (nh, "body_controllables/right_arm");
  body_controllables.torso = readParam<std::string>(nh, "body_controllables/torso");
  std::string frame_id = readParam<std::string>(nh, "frame_id");
  double update_period = readParam<double>(nh, "update_period");
  double server_timeout = readParam<double>(nh, "server_timeout");

  ControllerActionServer server(nh, "move", update_period, "/tf2_buffer_server", 
      frame_id, body_controllables, thresholds);
  try
  {
    server.start(ros::Duration(server_timeout));
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
    return 0;
  }

  ros::spin();

  return 0;
}
