/*
* Copyright (C) 2017 Georg Bartels <georg.bartels@cs.uni-bremen.de>
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
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <giskard_msgs/WholeBodyAction.h>
#include <giskard_core/giskard_core.hpp>
#include <giskard_ros/ros_utils.hpp>

namespace giskard_ros
{
  class QPControllerTrajectory
  {
    public:
      QPControllerTrajectory(const ros::NodeHandle& nh, const std::string& joint_traj_act_name,
                             const std::string& giskard_act_name,
                             const ros::Duration& act_server_timeout) :
          nh_(nh),
          js_sub_( nh_.subscribe("joint_states", 1, &QPControllerTrajectory::js_callback, this) ),
          joint_traj_act_( nh_, joint_traj_act_name, true ),
          giskard_act_( nh_, giskard_act_name, boost::bind(&QPControllerTrajectory::goal_callback, this, _1), false )
      {
        if (!robot_model_.initParam("/robot_description"))
          throw std::runtime_error("Could not read urdf from parameter server at '/robot_description'.");

        root_link_ = readParam<std::string>(nh_, "root_link");

        read_joint_weights();

        read_joint_velocity_thresholds();

        if (!joint_traj_act_.waitForServer(act_server_timeout))
            throw std::runtime_error("Waited for action server '" + joint_traj_act_name + "' for " +
                                      std::to_string(act_server_timeout.toSec()) + "s. Aborting.");


        giskard_act_.start();
      }

    protected:
      // ROS communication
      ros::NodeHandle nh_;
      ros::Subscriber js_sub_;
      actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> joint_traj_act_;
      actionlib::SimpleActionServer<giskard_msgs::WholeBodyAction> giskard_act_;

      // internal state
      std::map<std::string, double> current_joint_state_;
      urdf::Model robot_model_;
      std::string root_link_;
      std::map<std::string, double> joint_weights_, joint_velocity_thresholds_;

      void js_callback(const sensor_msgs::JointState::ConstPtr& msg)
      {
        if (msg->name.size() != msg->position.size())
          throw std::runtime_error("Received a joint state message with position and name fields of different length.");

        current_joint_state_.clear();
        for (size_t i=0; i<msg->name.size(); ++i)
          current_joint_state_.insert(std::make_pair(msg->name[i], msg->position[i]));
      }

      void goal_callback(const giskard_msgs::WholeBodyGoalConstPtr& goal)
      {
        ROS_DEBUG("Received a new goal.");
        std::cout << *goal << std::endl;
        try
        {
          giskard_core::QPControllerProjection projection = create_projection(*goal);
          projection.run(current_joint_state_);
          joint_traj_act_.cancelAllGoals();
          // TODO: periodically check for preemption

          joint_traj_act_.sendGoalAndWait(calc_trajectory_goal(), calc_trajectory_duration());
          if (joint_traj_act_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            giskard_act_.setSucceeded(giskard_msgs::WholeBodyResult());
          else
            giskard_act_.setAborted(giskard_msgs::WholeBodyResult(),
                "Trajectory execution failed with: " + joint_traj_act_.getState().getText());

        }
        catch (const std::exception& e)
        {
          ROS_ERROR("%s", e.what());
          giskard_act_.setAborted(giskard_msgs::WholeBodyResult(), e.what());
        }

      }

      giskard_core::QPControllerProjection create_projection(const giskard_msgs::WholeBodyGoal& goal)
      {
        // TODO: implement me
//      giskard_core::QPControllerParams params = get_qp_controller_params(*goal);
//      giskard_core::QPControllerSpecGenerator generator(params);
//      giskard_core::QPController controller();
//      giskard_core::QPControllerProjectionParams params(period, observable_names, convergence_thresholds, min_traj_points, max_traj_points, nWSR);
//      giskard_core::QPControllerProjection projection(controller, params);
//      projection.run(current_joint_state_);
//      joint_traj_act_.sendGoalAndWait(get_trajectory_goal(), get_trajectory_duration());
        giskard_core::QPControllerParams gen_params = create_generator_params(goal);
        giskard_core::QPController controller;
        giskard_core::QPControllerProjectionParams params(0.01, {}, {}, 10, 1000, 100);
        return giskard_core::QPControllerProjection(controller, params);
      }

      control_msgs::FollowJointTrajectoryGoal calc_trajectory_goal()
      {
        // TODO: implement me
        return control_msgs::FollowJointTrajectoryGoal();
      }

      ros::Duration calc_trajectory_duration()
      {
        // TODO: implement me
        return ros::Duration(5.0);
      }

      giskard_core::QPControllerParams create_generator_params(const giskard_msgs::WholeBodyGoal& goal)
      {
        // TODO: fill control_params based on goal
        const std::map<std::string, giskard_core::ControlParams> control_params;

        return giskard_core::QPControllerParams(robot_model_, root_link_, joint_weights_,
            joint_velocity_thresholds_, control_params);
      }

      void read_joint_weights()
      {
        joint_weights_.clear();

        try {
          joint_weights_ = readParam< std::map<std::string, double> >(nh_, "joint_weights");
        }
        catch (const std::exception& e) {
          ROS_WARN("%s", e.what());
        }

        joint_weights_.insert(std::make_pair(giskard_core::Robot::default_joint_weight_key(), readParam<double>(nh_, giskard_core::Robot::default_joint_weight_key())));
      }

      void read_joint_velocity_thresholds()
      {
        joint_velocity_thresholds_.clear();

        try {
          joint_velocity_thresholds_ = readParam< std::map<std::string, double> >(nh_, "joint_velocity_thresholds");
        }
        catch (const std::exception& e) {
          ROS_WARN("%s", e.what());
        }

        joint_velocity_thresholds_.insert(std::make_pair(giskard_core::Robot::default_joint_velocity_key(), readParam<double>(nh_, giskard_core::Robot::default_joint_velocity_key())));
      }
  };
}

int main(int argc, char **argv)
{
  using namespace giskard_ros;
  ros::init(argc, argv, "qp_controller");
  ros::NodeHandle nh("~");

  try
  {
    QPControllerTrajectory controller(nh, "follow_joint_trajectory", "command", ros::Duration(2.0));
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
    return 0;
  }

  return 0;
}