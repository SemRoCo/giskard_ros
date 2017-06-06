/*
* Copyright (C) 2015-2017 Jannik Buckelo <jannikbu@cs.uni-bremen.de>,
*                         Georg Bartels <georg.bartels@cs.uni-bremen.de>
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

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <giskard_msgs/WholeBodyCommand.h>
#include <giskard_msgs/ControllerFeedback.h>
#include <giskard_msgs/SemanticFloat64Array.h>
#include <yaml-cpp/yaml.h>
#include <giskard_core/giskard_core.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <boost/lexical_cast.hpp>
#include <giskard_ros/ros_utils.hpp>
#include <giskard_ros/utils.hpp>
#include <giskard_ros/watchdog.hpp>

#include <giskard_ros/command_utils.hpp>
#include <giskard_ros/conversions.hpp>
#include <giskard_ros/whole_body_controller.hpp>

namespace giskard_ros
{
      void ControllerContext::set_controller(const giskard_core::QPController& controller)
      {
        controller_ = controller;

        feedback_.commands.resize(controller_.num_controllables());
        for (size_t i=0; i<controller_.num_controllables(); ++i)
          feedback_.commands[i].semantics = controller_.get_controllable_names()[i];
        vel_command_.data = feedback_.commands;

        feedback_.slacks.resize(controller_.num_soft_constraints());
        for (size_t i=0; i<controller_.num_soft_constraints(); ++i)
          feedback_.slacks[i].semantics = controller_.get_soft_constraint_names()[i];

        state_ = Eigen::VectorXd::Zero(controller_.num_observables());
      }

      const giskard_core::QPController& ControllerContext::get_controller() const
      {
        return controller_;
      }

      void ControllerContext::set_command(const giskard_msgs::WholeBodyCommand& command)
      {
        feedback_.current_command_hash = 
          giskard_ros::calculateHash<giskard_msgs::WholeBodyCommand>(command);
        feedback_.current_command = command;

        feedback_.convergence_features.clear();
        switch (command.type)
        {
          case (giskard_msgs::WholeBodyCommand::STANDARD_CONTROLLER):
            feedback_.convergence_features.reserve(
                command.left_ee.convergence_thresholds.size() +
                command.right_ee.convergence_thresholds.size());
            feedback_.convergence_features.insert
              (feedback_.convergence_features.end(), 
               command.left_ee.convergence_thresholds.begin(), 
               command.left_ee.convergence_thresholds.end());
            feedback_.convergence_features.insert
              (feedback_.convergence_features.end(), 
               command.right_ee.convergence_thresholds.begin(), 
               command.right_ee.convergence_thresholds.end());
            break;
          case (giskard_msgs::WholeBodyCommand::YAML_CONTROLLER):
            feedback_.convergence_features = command.convergence_thresholds;
            break;
          default:
            throw std::runtime_error("Got unknown type of whole-body controller.");
        }

        if (command.type == giskard_msgs::WholeBodyCommand::STANDARD_CONTROLLER)
        {
          switch (command.left_ee.type)
          {
            case giskard_msgs::ArmCommand::JOINT_GOAL:
              state_.block(controller_.num_controllables(), 0, command.left_ee.goal_configuration.size(), 1) =
                to_eigen(command.left_ee.goal_configuration);
              break;
            case giskard_msgs::ArmCommand::CARTESIAN_GOAL:
               state_.block(controller_.num_controllables(), 0, 6, 1) =
                to_eigen(command.left_ee.goal_pose.pose);
              break;
            default:
              throw std::runtime_error("Received command of unknown type for left arm.");
          }

          size_t offset = (command.left_ee.type == giskard_msgs::ArmCommand::JOINT_GOAL ? 7 : 6);
          switch (command.right_ee.type)
          {
            case giskard_msgs::ArmCommand::JOINT_GOAL:
              state_.block(controller_.num_controllables() + offset, 0, command.right_ee.goal_configuration.size(), 1) =
                to_eigen(command.right_ee.goal_configuration);
              break;
            case giskard_msgs::ArmCommand::CARTESIAN_GOAL:
              state_.block(controller_.num_controllables() + offset, 0, 6, 1) =
                to_eigen(command.right_ee.goal_pose.pose);
              break;
            default:
              throw std::runtime_error("Received command of unknown type for left arm.");
          }
        }
      }

      const giskard_msgs::WholeBodyCommand& ControllerContext::get_command() const
      {
        return get_feedback().current_command;
      }

      const giskard_msgs::ControllerFeedback& ControllerContext::get_feedback() const
      {
        return feedback_;
      }

      const giskard_msgs::SemanticFloat64Array& ControllerContext::get_vel_command() const
      {
        return vel_command_;
      }

      bool ControllerContext::update(const sensor_msgs::JointState& msg, int nWSR)
      {
        set_joint_state(msg);
      
        if (controller_.update(state_, nWSR))
        {
          // fill feedback
          feedback_.header = msg.header;
          for (size_t i=0; i<feedback_.commands.size(); ++i)
            feedback_.commands[i].value = controller_.get_command()[i];
          for (size_t i=0; i<feedback_.slacks.size(); ++i)
            feedback_.slacks[i].value = controller_.get_slack()[i];
          for (size_t i=0; i<feedback_.convergence_features.size(); ++i)
            feedback_.convergence_features[i].value = 
              controller_.get_scope().find_double_expression(
                feedback_.convergence_features[i].semantics)->value();
          for (size_t i=0; i<feedback_.doubles.size(); ++i)
            feedback_.doubles[i].value = 
              controller_.get_scope().find_double_expression(
                feedback_.doubles[i].semantics)->value();
          for (size_t i=0; i<feedback_.vectors.size(); ++i)
              tf::vectorKDLToMsg(controller_.get_scope().find_vector_expression(
                    feedback_.vectors[i].semantics)->value(), feedback_.vectors[i].value);
          // fill command
          vel_command_.data = feedback_.commands;

          return true;
        }
        else 
          return false;
      }

      void ControllerContext::start_controller(
          const giskard_msgs::WholeBodyCommand& command,
          const WholeBodyControllerParams& params,
          const sensor_msgs::JointState& msg, 
          const std::string& name)
      {
        sanity_check(command, params);
        set_command(command);
        set_joint_state(msg);

        if (!controller_.start(state_, params.nWSR))
          throw std::runtime_error("Could not start " + name + " controller.");
      }

      void ControllerContext::set_joint_state(const sensor_msgs::JointState& msg)
      {
        // TODO: turn this into a map!
        // is there a more efficient way?
        for (size_t i=0; i < get_controller().num_controllables(); ++i)
          for (size_t j=0; j < msg.name.size(); ++j)
            if (get_controller().get_controllable_names()[i].find(msg.name[j]) == 0)
              state_[i] = msg.position[j];
      }

      WholeBodyController::WholeBodyController(const ros::NodeHandle& nh): 
        nh_(nh), state_(WholeBodyControllerState::constructed) {}
      WholeBodyController::~WholeBodyController() {}

      void WholeBodyController::start()
      {
        ROS_DEBUG("Calling start.");
        if (state_ == WholeBodyControllerState::constructed)
        {
          init_parameters();
          watchdog_.setPeriod(ros::Duration(readParam<double>(nh_, "watchdog_period")));
          init_controller_contexts();
  
          feedback_pub_ = nh_.advertise<giskard_msgs::ControllerFeedback>("feedback", 1, true);
          velocity_pub_ = nh_.advertise<giskard_msgs::SemanticFloat64Array>("velocity_cmd", 1);
          joint_state_sub_ = nh_.subscribe("joint_states", 1, &WholeBodyController::joint_state_callback, this,
            ros::TransportHints().tcpNoDelay());

          state_ = WholeBodyControllerState::started;
        }
        ROS_DEBUG("Finished start.");
      }

      void WholeBodyController::joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg)
      {
        if (state_ == WholeBodyControllerState::started)
          process_first_joint_state(*msg);

        if (watchdog_.barking(msg->header.stamp))
          process_watchdog(msg->header);
        else
          process_regular_joint_state(*msg);

        last_joint_state_ = *msg;
      }

      void WholeBodyController::command_callback(const giskard_msgs::WholeBodyCommand::ConstPtr& msg)
      {
        size_t new_command_hash = 
          giskard_ros::calculateHash<giskard_msgs::WholeBodyCommand>(*msg);
      
        if(get_current_context().get_feedback().current_command_hash == new_command_hash)
        {
          watchdog_.kick(ros::Time::now());
        }
        else
          process_new_command(*msg);
      }

      // INTERNAL HELPER FUNCTIONS

      ControllerContext& WholeBodyController::get_current_context()
      {
        return get_context(current_controller_);
      }

      ControllerContext& WholeBodyController::get_context(const std::string& controller)
      {
        if(parameters_.controller_types.count(controller) == 0)
          throw std::runtime_error("Could not retrieve current controller with unknown name + '" + 
              controller + "'.");

        return contexts_[controller];
      }

      giskard_msgs::WholeBodyCommand WholeBodyController::complete_command(const giskard_msgs::WholeBodyCommand& new_command,
          const giskard_msgs::WholeBodyCommand& current_command)
      {
        if (new_command.type == giskard_msgs::WholeBodyCommand::YAML_CONTROLLER)
          return new_command;
        else
        {
          giskard_msgs::WholeBodyCommand completed_command = new_command;
          if (completed_command.right_ee.type == giskard_msgs::ArmCommand::IGNORE_GOAL)
            completed_command.right_ee = current_command.right_ee;
          if (completed_command.left_ee.type == giskard_msgs::ArmCommand::IGNORE_GOAL)
            completed_command.left_ee = current_command.left_ee;
          return completed_command;
        }
      }

      std::string WholeBodyController::infer_controller(const giskard_msgs::WholeBodyCommand& msg)
      {
        switch (msg.type)
        {
          case (giskard_msgs::WholeBodyCommand::STANDARD_CONTROLLER):
            // more detailed selection in next switch-case-statement
            break;
          case (giskard_msgs::WholeBodyCommand::YAML_CONTROLLER):
            return "yaml";
          default:
            throw std::runtime_error("Got unknown controller type for whole-body.");
        }

        switch (msg.left_ee.type)
        {
          case (giskard_msgs::ArmCommand::JOINT_GOAL):
            switch (msg.right_ee.type)
            {
              case (giskard_msgs::ArmCommand::JOINT_GOAL):
                return "joint_joint";
              case (giskard_msgs::ArmCommand::CARTESIAN_GOAL):
                return "joint_cart";
              default:
                throw std::runtime_error("Got unknown controller type for right arm.");
            }
            break;
          case (giskard_msgs::ArmCommand::CARTESIAN_GOAL):
            switch (msg.right_ee.type)
            {
              case (giskard_msgs::ArmCommand::JOINT_GOAL):
                return "cart_joint";
              case (giskard_msgs::ArmCommand::CARTESIAN_GOAL):
                return "cart_cart";
              default:
                throw std::runtime_error("Got unknown controller type for right arm.");
            }
            break;
          default:
            throw std::runtime_error("Got unknown controller type for left arm.");
        }
      }

      void WholeBodyController::process_new_command(const giskard_msgs::WholeBodyCommand& msg)
      {
        giskard_msgs::WholeBodyCommand new_command = 
          complete_command(msg, get_current_context().get_command());
        current_controller_ = infer_controller(new_command);
        if (current_controller_.compare("yaml") == 0)
          init_and_start_yaml_controller(msg);
        else
          get_current_context().set_command(new_command);
        watchdog_.kick(ros::Time::now());
      }

      void WholeBodyController::init_and_start_yaml_controller(const giskard_msgs::WholeBodyCommand& msg)
      {
        YAML::Node node = YAML::Load(msg.yaml_spec);
        giskard_core::QPControllerSpec spec = node.as< giskard_core::QPControllerSpec >();
        giskard_core::QPController controller = giskard_core::generate(spec);
        for (size_t i=0; i<parameters_.joint_names.size(); ++i)
          if (controller.get_controllable_names()[i].find(parameters_.joint_names[i]) != 0)
            throw std::runtime_error("Name of joint '" + parameters_.joint_names[i] + 
                "' and controllable '" + controller.get_controllable_names()[i] + 
                "' did not match.");
        get_current_context().set_controller(controller);
        get_current_context().start_controller(msg, parameters_, last_joint_state_, "yaml");
      }

      void WholeBodyController::init_parameters()
      {
        parameters_.nWSR = readParam<int>(nh_, "nWSR");
        // TODO: extract joint_names from controller description
        parameters_.joint_names = readParam< std::vector<std::string> >(nh_, "joint_names");
        // TODO: harmonize with bodypart notation from other nodes?
        parameters_.l_arm_names = readParam< std::vector<std::string> >(nh_, "l_arm_names");
        parameters_.r_arm_names = readParam< std::vector<std::string> >(nh_, "r_arm_names");
        parameters_.frame_id = readParam< std::string >(nh_, "frame_id");
        parameters_.l_fk_name = readParam< std::string >(nh_, "l_fk_name");
        parameters_.r_fk_name = readParam< std::string >(nh_, "r_fk_name");
        parameters_.controller_types = {"cart_cart", "joint_cart", "cart_joint", "joint_joint", "yaml"};
      }

      void WholeBodyController::init_controller_contexts()
      {
        std::map<std::string, std::string> controller_descriptions =
          read_controller_descriptions();

        for(std::set<std::string>::const_iterator it=parameters_.controller_types.begin();
            it!=parameters_.controller_types.end(); ++it)
        {
          if (it->compare("yaml") != 0)
          {
            YAML::Node node = YAML::Load(controller_descriptions.at(*it));
            giskard_core::QPControllerSpec spec = node.as< giskard_core::QPControllerSpec >();
            giskard_core::QPController controller = giskard_core::generate(spec);
            for (size_t i=0; i<parameters_.joint_names.size(); ++i)
              if (controller.get_controllable_names()[i].find(parameters_.joint_names[i]) != 0)
                throw std::runtime_error("Name of joint '" + parameters_.joint_names[i] + 
                    "' and controllable '" + controller.get_controllable_names()[i] + 
                    "' did not match.");
  
            ControllerContext context;
            context.set_controller(controller);
            contexts_.insert( std::pair<std::string, ControllerContext>(*it, context));
          }
        }
      }

      std::map<std::string, std::string> WholeBodyController::read_controller_descriptions()
      {
        std::map<std::string, std::string> result = 
          readParam< std::map<std::string, std::string> >(nh_, "controller_descriptions");
        for (std::set<std::string>::const_iterator it=parameters_.controller_types.begin();
             it!=parameters_.controller_types.end(); ++it)
          if(it->compare("yaml") != 0 && result.find(*it) == result.end())
            throw std::runtime_error("Could not find controller description for '" + *it + "'.");
        return result;
      }

      void WholeBodyController::process_first_joint_state(const sensor_msgs::JointState& msg)
      {
        contexts_.at("joint_joint").start_controller( 
            init_joint_joint_command(msg, parameters_), 
            parameters_, msg, "joint_joint");
        contexts_.at("cart_joint").start_controller(
            init_cart_joint_command(msg, eval_fk(parameters_.l_fk_name, msg), parameters_), 
            parameters_, msg, "cart_joint");
        contexts_.at("joint_cart").start_controller(
            init_joint_cart_command(msg, eval_fk(parameters_.r_fk_name, msg), parameters_),
            parameters_, msg, "joint_cart");
        contexts_.at("cart_cart").start_controller(
            init_cart_cart_command(msg.header.stamp, parameters_.frame_id,
              eval_fk(parameters_.l_fk_name, msg), eval_fk(parameters_.r_fk_name, msg)), 
            parameters_, msg, "cart_cart");
        state_ = WholeBodyControllerState::running;
        current_controller_ = "cart_cart";
        goal_sub_ = nh_.subscribe("goal", 1, &WholeBodyController::command_callback, this);
      }

      void WholeBodyController::process_regular_joint_state(const sensor_msgs::JointState& msg)
      {
        ControllerContext& context = get_current_context();
        if (context.update(msg, parameters_.nWSR))
        {
          velocity_pub_.publish(context.get_vel_command());
          feedback_pub_.publish(context.get_feedback());
        }
        else
          throw std::runtime_error("Update of controller '" + current_controller_ + "' failed.");
      }

      void WholeBodyController::process_watchdog(const std_msgs::Header& header)
      {
        giskard_msgs::ControllerFeedback feedback;
        giskard_msgs::SemanticFloat64Array command;
        for (size_t i=0; i<parameters_.joint_names.size(); ++i)
        {
          giskard_msgs::SemanticFloat64 msg;
          msg.value = 0.0;
          msg.semantics = parameters_.joint_names[i];
          feedback.commands.push_back(msg);
        }
        feedback.header = header;
        feedback.watchdog_active = true;
        feedback_pub_.publish(feedback);

        command.data = feedback.commands;
        velocity_pub_.publish(command);
      }

      KDL::Frame WholeBodyController::eval_fk(const std::string& fk_name, 
          const sensor_msgs::JointState& msg)
      {
        const giskard_core::QPController& controller = get_context("cart_cart").get_controller();
        KDL::Expression<KDL::Frame>::Ptr fk = controller.get_scope().find_frame_expression(fk_name);
        std::set<int> deps;
        fk->getDependencies(deps);
        for (std::set<int>::const_iterator it=deps.begin(); it!=deps.end(); ++it)
          for (size_t i=0; i<msg.name.size(); ++i)
            if (controller.get_controllable_names()[*it].find(msg.name[i]) == 0)
              fk->setInputValue(*it, msg.position[i]);

        return fk->value();
      }
}
