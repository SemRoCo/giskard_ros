/*
* Copyright (C) 2015, 2016 Jannik Buckelo <jannikbu@cs.uni-bremen.de>,
* Georg Bartels <georg.bartels@cs.uni-bremen.de>
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

#ifndef __GISKARD_WHOLE_BODY_CONTROLLER_HPP__
#define __GISKARD_WHOLE_BODY_CONTROLLER_HPP__

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <giskard_msgs/WholeBodyCommand.h>
#include <giskard_msgs/ControllerFeedback.h>
#include <giskard_msgs/SemanticFloat64Array.h>
#include <yaml-cpp/yaml.h>
#include <giskard/giskard.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <boost/lexical_cast.hpp>
#include <giskard_examples/ros_utils.hpp>
#include <giskard_examples/utils.hpp>
#include <giskard_examples/watchdog.hpp>

namespace giskard_examples
{
  // TODO: move this somewhere else
  inline Eigen::VectorXd to_eigen(const std::vector<double>& v)
  {
    Eigen::VectorXd result(v.size());
    for (size_t i=0; i<v.size(); ++i)
      result[i] = v[i];
    return result;
  }

  // TODO: move this somewhere else
  inline Eigen::VectorXd to_eigen(const geometry_msgs::Pose& p)
  {
    Eigen::VectorXd result(6);
    result[0] = p.position.x;
    result[1] = p.position.y;
    result[2] = p.position.z;
  
    KDL::Rotation rot;
    tf::quaternionMsgToKDL(p.orientation, rot);
    rot.GetEulerZYX(result[3], result[4], result[5]);

    return result;
  }

  class ControllerContext
  {
    private:
      giskard::QPController controller_;
      Eigen::VectorXd state_;
      giskard_msgs::ControllerFeedback feedback_;
      giskard_msgs::SemanticFloat64Array vel_command_;

    public:
      void set_controller(const giskard::QPController& controller);

      bool update(const sensor_msgs::JointState& msg, int nWSR);

      bool start(int nWSR);

      void set_joint_state(const sensor_msgs::JointState& msg);

      const giskard::QPController& get_controller() const;

      void set_command(const giskard_msgs::WholeBodyCommand& command);

      const giskard_msgs::WholeBodyCommand& get_command() const;

      const giskard_msgs::ControllerFeedback& get_feedback() const;

      const giskard_msgs::SemanticFloat64Array& get_vel_command() const;
  };

  class WholeBodyControllerParams
  {
    public:
      std::string frame_id, l_fk_name, r_fk_name;
      std::vector< std::string > joint_names, l_arm_names, r_arm_names;
      std::set< std::string > controller_types;
      int nWSR;
  };

  enum class WholeBodyControllerState { constructed, started, running };

  class WholeBodyController
  {
    public:
      WholeBodyController(const ros::NodeHandle& nh); 
      ~WholeBodyController();

      void start();

    private:
      ros::NodeHandle nh_;
      ros::Publisher velocity_pub_, feedback_pub_;
      ros::Subscriber goal_sub_, joint_state_sub_;

      std::map< std::string, ControllerContext > contexts_;
      Watchdog<ros::Time, ros::Duration> watchdog_;
      std::string current_controller_;
      WholeBodyControllerParams parameters_;
      WholeBodyControllerState state_;
      sensor_msgs::JointState last_joint_state_;

      // CALLBACKS
      void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg);

      void command_callback(const giskard_msgs::WholeBodyCommand::ConstPtr& msg);

      // INTERNAL HELPER FUNCTIONS
      // TODO: check whether these could move somewhere else

      ControllerContext& get_current_context();

      ControllerContext& get_context(const std::string& controller);

      giskard_msgs::WholeBodyCommand complete_command(const giskard_msgs::WholeBodyCommand& new_command,
          const giskard_msgs::WholeBodyCommand& current_command);

      std::string infer_controller(const giskard_msgs::WholeBodyCommand& msg);

      void process_new_command(const giskard_msgs::WholeBodyCommand& msg);

      void init_and_start_yaml_controller(const giskard_msgs::WholeBodyCommand& msg);

      void init_parameters();

      void init_controller_contexts();

      std::map<std::string, std::string> read_controller_descriptions();

      void process_first_joint_state(const sensor_msgs::JointState& msg);

      void process_regular_joint_state(const sensor_msgs::JointState& msg);

      void process_watchdog(const std_msgs::Header& header);

      giskard_msgs::ArmCommand init_arm_joint_command(const sensor_msgs::JointState& msg, const std::vector<std::string>& joint_names);
      
      giskard_msgs::ArmCommand init_arm_cart_command(const sensor_msgs::JointState& msg, 
          const std::string& frame_id, const std::string& fk_name);

      giskard_msgs::WholeBodyCommand init_joint_joint_command (const sensor_msgs::JointState& msg);

      giskard_msgs::WholeBodyCommand init_cart_cart_command (const sensor_msgs::JointState& msg);

      giskard_msgs::WholeBodyCommand init_cart_joint_command (const sensor_msgs::JointState& msg);

      giskard_msgs::WholeBodyCommand init_joint_cart_command (const sensor_msgs::JointState& msg);

      void sanity_check(const std::vector<double>& v, const std::vector<std::string>& joint_names, const std::string& name);
         
      void sanity_check(const geometry_msgs::PoseStamped& msg, const std::string& name);

      void sanity_check(const giskard_msgs::ArmCommand& msg, 
          const std::vector<std::string>& joint_names, const std::string& name);
 
      void sanity_check(const giskard_msgs::WholeBodyCommand& command);

      void start_controller(ControllerContext& context, 
          const giskard_msgs::WholeBodyCommand& command,
          const sensor_msgs::JointState& msg, 
          const std::string& name);
  };
}

#endif // __GISKARD_WHOLE_BODY_CONTROLLER__HPP
