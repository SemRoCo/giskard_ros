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
#include <tf2_ros/buffer_client.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <giskard_msgs/WholeBodyAction.h>
#include <giskard_core/giskard_core.hpp>
#include <giskard_ros/ros_utils.hpp>
#include <giskard_ros/conversions.hpp>

namespace giskard_ros
{
  class QPControllerTrajectory
  {
    public:
      QPControllerTrajectory(const ros::NodeHandle& nh, const std::string& joint_traj_act_name,
                             const std::string& giskard_act_name, const ros::Duration& server_timeout,
                             const std::string& tf_ns) :
          nh_(nh),
          js_sub_( nh_.subscribe("joint_states", 1, &QPControllerTrajectory::js_callback, this) ),
          joint_traj_state_sub_( nh_.subscribe("joint_trajectory_controller_state", 1, &QPControllerTrajectory::joint_traj_state_callback, this) ),
          joint_traj_act_( nh_, joint_traj_act_name, true ),
          giskard_act_( nh_, giskard_act_name, boost::bind(&QPControllerTrajectory::goal_callback, this, _1), false ),
          tf_(std::make_shared<tf2_ros::BufferClient>(tf_ns))
      {
        if (!robot_model_.initParam("/robot_description"))
          throw std::runtime_error("Could not read urdf from parameter server at '/robot_description'.");

        root_link_ = readParam<std::string>(nh_, "root_link");

        sample_period_ = readParam<double>(nh_, "sample_period");
        read_joint_weights();

        read_joint_velocity_thresholds();

        if (!joint_traj_act_.waitForServer(server_timeout))
            throw std::runtime_error("Waited for action server '" + joint_traj_act_name + "' for " +
                                      std::to_string(server_timeout.toSec()) + "s. Aborting.");

        if(!tf_->waitForServer(server_timeout))
          throw std::runtime_error("Waited for TF2 BufferServer " + std::to_string(server_timeout.toSec()) + "s. Aborting.");

        giskard_act_.start();
      }

    protected:
      // ROS communication
      ros::NodeHandle nh_;
      ros::Subscriber js_sub_, joint_traj_state_sub_;
      actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> joint_traj_act_;
      actionlib::SimpleActionServer<giskard_msgs::WholeBodyAction> giskard_act_;
      std::shared_ptr<tf2_ros::BufferClient> tf_;

      // internal state
      std::map<std::string, double> current_joint_state_;
      trajectory_msgs::JointTrajectoryPoint joint_controller_state_;
      std::map<std::string, size_t> joint_controller_joint_name_index_;
      std::vector<std::string> joint_controller_joint_names_;

      // parameters
      urdf::Model robot_model_;
      std::string root_link_;
      std::map<std::string, double> joint_weights_, joint_velocity_thresholds_;
      double sample_period_;

      void js_callback(const sensor_msgs::JointState::ConstPtr& msg)
      {
        if (msg->name.size() != msg->position.size())
          throw std::runtime_error("Received a joint state message with position and name fields of different length.");

        current_joint_state_.clear();
        for (size_t i=0; i<msg->name.size(); ++i)
          current_joint_state_.insert(std::make_pair(msg->name[i], msg->position[i]));
      }

      void joint_traj_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
      {
        // copy the current state, i.e. joint positions, velocities, etc.
        joint_controller_state_ = msg->actual;

        // check whether the joint names in the state are different from what we internally hold
        bool joint_names_equal = false;
        if (msg->joint_names.size() == joint_controller_joint_name_index_.size())
          joint_names_equal = std::equal(msg->joint_names.begin(), msg->joint_names.end(), joint_controller_joint_names_.begin());

        if (!joint_names_equal)
        {
          // TODO: turn this into a debug
          ROS_INFO("Recalculating index of joint names of the joint controller.");
          joint_controller_joint_names_ = msg->joint_names;
          joint_controller_joint_name_index_.clear();
          for (size_t i=0; i<msg->joint_names.size(); ++i)
            joint_controller_joint_name_index_.insert(std::make_pair(msg->joint_names[i], i));
        }
      }

      size_t joint_goal_index(const std::string& joint_name) const
      {
        if (joint_controller_joint_name_index_.count(joint_name) == 0)
          throw std::runtime_error("Could not find joint controller index for joint '" + joint_name + "'.");

        return joint_controller_joint_name_index_.find(joint_name)->second;
      }

      void goal_callback(const giskard_msgs::WholeBodyGoalConstPtr& goal)
      {
        ros::Time start_time = ros::Time::now();
        ROS_INFO("Received a new goal.");
        try
        {
          giskard_core::QPControllerProjection projection = create_projection(*goal);
          ros::Time setup_complete = ros::Time::now();
          projection.run(get_observable_values(projection, *goal));
          ros::Time projection_complete = ros::Time::now();
          ROS_INFO_STREAM("Setup time: " << (setup_complete - start_time).toSec());
          ROS_INFO_STREAM("Projection time: " << (projection_complete - setup_complete).toSec());

          joint_traj_act_.cancelAllGoals();

          ROS_INFO_STREAM("Commanding trajectory with duration " << calc_trajectory_duration(projection));
          joint_traj_act_.sendGoal(calc_trajectory_goal(projection));
          ros::Duration traj_duration = calc_trajectory_duration(projection);
          ros::Time start_time = ros::Time::now();
          ros::Rate monitor_rate(10); // TODO: get this from the parameter server?

          while((ros::Time::now() - start_time) < traj_duration)
          {
            if (giskard_act_.isPreemptRequested() || !ros::ok())
            {
              giskard_act_.setPreempted(giskard_msgs::WholeBodyResult(), "Received preempt request.");
              joint_traj_act_.cancelAllGoals();
              return;
            }

            if (joint_traj_act_.getState() == actionlib::SimpleClientGoalState::ACTIVE ||
                joint_traj_act_.getState() == actionlib::SimpleClientGoalState::PENDING)
            {
              giskard_act_.setPreempted(giskard_msgs::WholeBodyResult(), "Joint trajectory action in unexpected state: " +
                      joint_traj_act_.getState().getText());
              joint_traj_act_.cancelAllGoals();
              return;
            }

            // TODO: re-run projection to emulate reactiveness
            monitor_rate.sleep();
          }

          ROS_INFO("Stopped monitoring trajectory goal.");
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

        ROS_INFO("Finished callback.");

      }

      giskard_core::QPControllerProjection create_projection(const giskard_msgs::WholeBodyGoal& goal)
      {
        if (goal.command.type == giskard_msgs::WholeBodyCommand::YAML_CONTROLLER)
          // TODO: implement me
          throw std::runtime_error("Command type YAML_CONTROLLER is not supported, yet.");

        giskard_core::QPControllerSpecGenerator gen(create_generator_params(goal));
        giskard_core::QPController controller = giskard_core::generate(gen.get_spec());
        // TODO: get these values from the parameter server
        std::map<std::string, double> joint_convergence_thresholds;
        for (auto const & controllable_name: gen.get_controllable_names())
          joint_convergence_thresholds.insert(std::make_pair(controllable_name, 0.001));
        joint_convergence_thresholds["torso_lift_joint"] = 0.0001;

        giskard_core::QPControllerProjectionParams params(sample_period_, gen.get_observable_names(), joint_convergence_thresholds, 10, 500, 100); // TODO: get these parameters from the server
        return giskard_core::QPControllerProjection(controller, params);
      }

      control_msgs::FollowJointTrajectoryGoal calc_trajectory_goal(const giskard_core::QPControllerProjection& projection) const
      {
        // TODO: turn this into a debug
        ROS_INFO_STREAM("Projection holds a trajectory with " << projection.get_position_trajectories().size() << " points.");

        // set up joint names
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.joint_names = joint_controller_joint_names_;

        // initialize memory of trajectory points in goal with current joint controller state
        // NOTE: this will lead to sane values for all the joints we are not controlling with the QPController
        goal.trajectory.points.resize(projection.get_position_trajectories().size(), joint_controller_state_);

        // fill trajectory points with desired values for the joint we control
        for (size_t i=0; i<projection.get_position_trajectories().size(); ++i)
        {
          trajectory_msgs::JointTrajectoryPoint& sample = goal.trajectory.points[i];
          sample.time_from_start = ros::Duration(sample_period_ * (i + 1));
          for (size_t j=0; j<projection.get_controllable_names().size(); ++j)
            sample.positions[joint_goal_index(projection.get_controllable_names()[j])] =
                projection.get_position_trajectories()[i](j);
          for (size_t j=0; j<projection.get_controllable_names().size(); ++j)
            sample.velocities[joint_goal_index(projection.get_controllable_names()[j])] =
                projection.get_velocity_trajectories()[i](j);
        }

        // set up start time
        goal.trajectory.header.stamp = ros::Time::now();

        return goal;
      }

      ros::Duration calc_trajectory_duration(const giskard_core::QPControllerProjection& projection) const
      {
        return ros::Duration(sample_period_ * projection.get_position_trajectories().size());
      }

      giskard_core::QPControllerParams create_generator_params(const giskard_msgs::WholeBodyGoal& goal)
      {
        if (goal.command.type == giskard_msgs::WholeBodyCommand::YAML_CONTROLLER)
          throw std::runtime_error("Cannot create QPControllerParams for command type YAML_CONTROLLER.");

        std::map<std::string, giskard_core::ControlParams> control_params;

        switch (goal.command.left_ee.type)
        {
          case giskard_msgs::ArmCommand::IGNORE_GOAL:
            break;
          case giskard_msgs::ArmCommand::JOINT_GOAL:
            // TODO: implement me
            throw std::runtime_error("Arm command type JOINT_GOAL is not supported, yet.");
          case giskard_msgs::ArmCommand::CARTESIAN_GOAL:
          {
            giskard_core::ControlParams trans3d_params;
            trans3d_params.type = giskard_core::ControlParams::Translation3D;
            trans3d_params.root_link = root_link_;
            trans3d_params.tip_link = "l_gripper_tool_frame"; // TODO: get these parameters from server
            trans3d_params.threshold_error = true;
            trans3d_params.threshold = 0.05;
            trans3d_params.p_gain = 3.0;
            trans3d_params.weight = 1.0;

            giskard_core::ControlParams rot3d_params;
            rot3d_params.type = giskard_core::ControlParams::Rotation3D;
            rot3d_params.root_link = root_link_;
            rot3d_params.tip_link = "l_gripper_tool_frame"; // TODO: get these parameters from server
            rot3d_params.threshold_error = true;
            rot3d_params.threshold = 0.1;
            rot3d_params.p_gain = 3.0;
            rot3d_params.weight = 1.0;

            // TODO: put these names somewhere central
            control_params.insert(std::make_pair("left_arm_translation3d", trans3d_params));
            control_params.insert(std::make_pair("left_arm_rotation3d", rot3d_params));
            break;
          }
          default:
            std::runtime_error("Received unknown arm command type: " + std::to_string(goal.command.left_ee.type));
        }

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
          joint_velocity_thresholds_ = readParam< std::map<std::string, double> >(nh_, "joint_velocity_limits");
        }
        catch (const std::exception& e) {
          ROS_WARN("%s", e.what());
        }

        joint_velocity_thresholds_.insert(std::make_pair(giskard_core::Robot::default_joint_velocity_key(), readParam<double>(nh_, giskard_core::Robot::default_joint_velocity_key())));
      }

      std::map<std::string, double> get_observable_values(const giskard_core::QPControllerProjection& projection,
          const giskard_msgs::WholeBodyGoal& goal)
      {
        std::map<std::string, double> result = current_joint_state_;

        switch (goal.command.left_ee.type)
        {
          case giskard_msgs::ArmCommand::IGNORE_GOAL:
            break;
          case giskard_msgs::ArmCommand::JOINT_GOAL:
            // TODO: implement me
            throw std::runtime_error("Arm command type JOINT_GOAL is not supported, yet.");
          case giskard_msgs::ArmCommand::CARTESIAN_GOAL:
          {
            // TODO: get this timeout from somewhere
            // TODO: refactor this into somewhere a bit prettier ;)
            geometry_msgs::PoseStamped transformed_goal_pose;
            tf_->transform(goal.command.left_ee.goal_pose, transformed_goal_pose, root_link_, ros::Duration(0.1));
            Eigen::Vector3d trans_tmp = to_eigen(transformed_goal_pose.pose.position);
            for (size_t i=0; i<trans_tmp.rows(); ++i)
              result.insert(std::make_pair(
                  giskard_core::QPControllerSpecGenerator::create_input_name("left_arm_translation3d", giskard_core::QPControllerSpecGenerator::translation3d_names()[i]),
                  trans_tmp(i)));

            KDL::Rotation goal_rot = to_kdl(transformed_goal_pose.pose.orientation);
            KDL::Vector axis;
            double angle = goal_rot.GetRotAngle(axis);
            Eigen::Vector4d rot_tmp = {axis.x(), axis.y(), axis.z(), angle};
            for (size_t i=0; i<rot_tmp.rows(); ++i)
              result.insert(std::make_pair(
                  giskard_core::QPControllerSpecGenerator::create_input_name("left_arm_rotation3d", giskard_core::QPControllerSpecGenerator::rotation3d_names()[i]),
                  rot_tmp(i)));

            break;
          }
          default:
            std::runtime_error("Received unknown arm command type: " + std::to_string(goal.command.left_ee.type));
        }

        return result;
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
    QPControllerTrajectory controller(nh, "follow_joint_trajectory", "command", ros::Duration(2.0), "/tf2_buffer_server");
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
    return 0;
  }

  return 0;
}