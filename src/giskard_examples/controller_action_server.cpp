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

  inline std::map<std::string, giskard_msgs::SemanticFloat64> toIndex2(const std::vector<giskard_msgs::SemanticFloat64>& msgs)
  {
    std::map<std::string, giskard_msgs::SemanticFloat64> index;
    for (size_t i=0; i<msgs.size(); ++i)
      index.insert(std::pair<std::string, giskard_msgs::SemanticFloat64>(msgs[i].semantics, msgs[i]));

    return index;
  }

  template<class T>
  inline T exception_lookup(const std::map<std::string, T>& index, const std::string& name)
  {
    typename std::map<std::string, T>::const_iterator it = index.find(name);
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
      result = maxAbsValue(result, exception_lookup<double>(index, search_names[i]));
    
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
      ros::Duration motion_old_;
      std::map<std::string, double> cartesian_;
      std::map<std::string, double> bodypart_moves_;
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
    std::map<std::string, giskard_msgs::SemanticFloat64> doubles_index = toIndex2(msg.doubles);

    result.state.left_arm_max_vel = 
      lookupMaxAbsValue(command_index, body_controllables.left_arm);
    result.state.right_arm_max_vel = 
      lookupMaxAbsValue(command_index, body_controllables.right_arm);
    result.state.torso_vel = 
      exception_lookup<double>(command_index, body_controllables.torso);
    result.state.motion_started = 
      running_command_hash == current_command_hash;
    result.state.motion_old = result.state.running_time > thresholds.motion_old_;
    result.state.torso_moving =
      std::abs(result.state.torso_vel) > std::abs(exception_lookup<double>(thresholds.bodypart_moves_, "torso"));
    result.state.left_arm_moving =
      std::abs(result.state.left_arm_max_vel) > std::abs(exception_lookup<double>(thresholds.bodypart_moves_, "left_arm"));
    result.state.right_arm_moving =
      std::abs(result.state.right_arm_max_vel) > std::abs(exception_lookup<double>(thresholds.bodypart_moves_, "right_arm"));
    for (std::map<std::string, double>::const_iterator it=thresholds.cartesian_.begin(); it!=thresholds.cartesian_.end(); ++it)
    {
      giskard_msgs::SemanticFloat64 feature_value = 
        exception_lookup<giskard_msgs::SemanticFloat64>(doubles_index, it->first);
      result.state.feature_values.push_back(feature_value);

      giskard_msgs::SemanticBool flag;
      flag.semantics = it->first;
      flag.value = (std::abs(feature_value.value) < std::abs(it->second));
      result.state.feature_flags.push_back(flag);
    }

    return result;
  }

  inline bool motionFinished(const giskard_msgs::WholeBodyFeedback& msg)
  {
    bool result = msg.state.motion_started && msg.state.motion_old && 
      !msg.state.torso_moving && !msg.state.left_arm_moving && !msg.state.right_arm_moving;

    for (size_t i=0; i<msg.state.feature_flags.size(); ++i)
      if (result)
        result &= msg.state.feature_flags[i].value;
      else
        return false;

    return result;
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
          const std::string& tf_ns, const std::string& frame_id, const std::string l_frame_id,
          const std::string r_frame_id, const WholeBodyControllables& body_controllables,
          const FeedbackThresholds& thresholds) : 
        nh_(nh), server_(nh, name, boost::bind(&ControllerActionServer::execute, this, _1), false),
        period_(ros::Duration(period)), frame_id_(frame_id), l_frame_id_(l_frame_id),
        r_frame_id_(r_frame_id), tf_(std::make_shared<tf2_ros::BufferClient>(tf_ns)),
        body_controllables_(body_controllables), thresholds_(thresholds)
      {}

      ~ControllerActionServer() {}

      void start(const ros::Duration& timeout)
      {
        if(!tf_->waitForServer(timeout))
          throw std::runtime_error("Wait for TF2 BufferServer timed out. Aborting.");

        initLastCommand();

        command_pub_ = nh_.advertise<giskard_msgs::WholeBodyCommand>("command", 1);
        feedback_sub_ = nh_.subscribe("feedback", 1, &ControllerActionServer::feedback, this);

        server_.start();
      }

    private:
      ros::NodeHandle nh_;
      ros::Subscriber feedback_sub_;
      ros::Publisher command_pub_;
      actionlib::SimpleActionServer<giskard_msgs::WholeBodyAction> server_;
      std::shared_ptr<tf2_ros::BufferClient> tf_;
      giskard_msgs::ControllerFeedback controller_feedback_;
      giskard_msgs::WholeBodyFeedback action_feedback_;
      giskard_msgs::WholeBodyCommand last_command_;
      size_t current_command_hash_;
      ros::Duration period_;
      ros::Time motion_start_time_;
      std::string frame_id_, l_frame_id_, r_frame_id_;
      WholeBodyControllables body_controllables_;
      FeedbackThresholds thresholds_;
      std::map< std::string, double> thresh_map_;

      void feedback(const giskard_msgs::ControllerFeedback::ConstPtr& msg)
      {
        if (!msg->watchdog_active && server_.isActive())
        {
          action_feedback_ = calculateFeedback(*msg, motion_start_time_, 
              body_controllables_, thresholds_, current_command_hash_,
              current_command_hash_);
          server_.publishFeedback(action_feedback_);
        }
      }

      void execute(const giskard_msgs::WholeBodyGoalConstPtr& goal)
      {
        ROS_DEBUG("Received a new motion goal.");

        giskard_msgs::WholeBodyCommand current_command;
        try
        {
          current_command = processCommand(goal->command, last_command_);
        }
        catch(tf2::TransformException& ex)
        {
          ROS_ERROR("Could not transform goal. Error: %s", ex.what());
          server_.setAborted(giskard_msgs::WholeBodyResult(), ex.what());
          return;
        }
 
        motion_start_time_ = ros::Time::now();

        current_command_hash_ = calculateHash<giskard_msgs::WholeBodyCommand>(current_command);
        action_feedback_ = giskard_msgs::WholeBodyFeedback();

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
            last_command_ = current_command;
            period_.sleep();
          }
        }
        while(!motionFinished(action_feedback_));

        server_.setSucceeded(calculateResult(action_feedback_));
      }

      giskard_msgs::WholeBodyCommand processCommand(const giskard_msgs::WholeBodyCommand& new_command,
          const giskard_msgs::WholeBodyCommand& old_command)
      {
        giskard_msgs::WholeBodyCommand processed_command = new_command;
        std::string err_string;
        // FIXME: get this timeout from somewhere
        if(new_command.left_ee.type == giskard_msgs::ArmCommand::CARTESIAN_GOAL)
          tf_->transform(new_command.left_ee.goal_pose, processed_command.left_ee.goal_pose, frame_id_, ros::Duration(0.1));
        else
          processed_command.left_ee = old_command.left_ee;

        if(new_command.right_ee.type == giskard_msgs::ArmCommand::CARTESIAN_GOAL)
          tf_->transform(new_command.right_ee.goal_pose, processed_command.right_ee.goal_pose, frame_id_, ros::Duration(0.1));
        else
          processed_command.right_ee = old_command.right_ee;

        return processed_command;
      }

      giskard_msgs::ArmCommand initArmCommand(const std::string& ee_frame_id, const ros::Time& stamp)
      {
        giskard_msgs::ArmCommand result;
        result.type = giskard_msgs::ArmCommand::CARTESIAN_GOAL;
        result.goal_pose.header.stamp = stamp;
        result.goal_pose.header.frame_id = ee_frame_id;
        result.goal_pose.pose.orientation.w = 1.0;
        std::string err_string;
        if (!tf_->canTransform(ee_frame_id, frame_id_, stamp, ros::Duration(2), &err_string))
          throw std::runtime_error(err_string);

        tf_->transform(result.goal_pose, result.goal_pose, frame_id_);
        return result;
      }

      void initLastCommand()
      {
        giskard_msgs::WholeBodyCommand init_command;
        ros::Time now = ros::Time(0);
        init_command.left_ee = initArmCommand(l_frame_id_, now);
        init_command.right_ee = initArmCommand(r_frame_id_, now);
        last_command_ = init_command;
      }
  };
}

int main(int argc, char **argv)
{
  using namespace giskard_examples;
  ros::init(argc, argv, "controller_action_server");
  ros::NodeHandle nh("~");

  FeedbackThresholds thresholds;
  thresholds.motion_old_ = ros::Duration(readParam<double>(nh, "thresholds/motion_old"));
  thresholds.bodypart_moves_ = readParam< std::map<std::string, double> >(nh, "thresholds/bodypart_moves");
  thresholds.cartesian_ = readParam< std::map<std::string, double> >(nh, "thresholds/cartesian");

  WholeBodyControllables body_controllables;
  body_controllables.left_arm = readParam< std::vector<std::string> >
    (nh, "body_controllables/left_arm");
  body_controllables.right_arm = readParam< std::vector<std::string> >
    (nh, "body_controllables/right_arm");
  body_controllables.torso = readParam<std::string>(nh, "body_controllables/torso");
  // TODO: refactor this into a container class
  std::string frame_id = readParam<std::string>(nh, "frame_id");
  std::string l_frame_id = readParam<std::string>(nh, "l_frame_id");
  std::string r_frame_id = readParam<std::string>(nh, "r_frame_id");
  double update_period = readParam<double>(nh, "update_period");
  double server_timeout = readParam<double>(nh, "server_timeout");

  ControllerActionServer server(nh, "move", update_period, "/tf2_buffer_server", 
      frame_id, l_frame_id, r_frame_id, body_controllables, thresholds);
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
