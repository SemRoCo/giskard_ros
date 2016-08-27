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
#include <actionlib/client/simple_action_client.h>
#include <yaml-cpp/yaml.h>
#include <giskard_examples/ros_utils.hpp>
#include <giskard_msgs/WholeBodyAction.h>

namespace giskard_examples
{
  std::map<std::string, double> parse_threshold_file(const std::string& file_content)
  {
    YAML::Node node = YAML::Load(file_content);
    std::map<std::string, double> result;
    for (YAML::const_iterator it=node.begin(); it!=node.end(); ++it)
      result.insert(
          std::pair<std::string, double>(
            it->first.as<std::string>(), it->second.as<double>()));
    return result;
  }

  class YAMLExecutive
  {
    public:
      YAMLExecutive(const ros::NodeHandle& nh, const std::string& action_name) :
          nh_(nh), client_(action_name, true) {}
      ~YAMLExecutive() {}
      void run()
      {
        ROS_INFO("Waiting for action server to start.");
        client_.waitForServer();
        ROS_INFO("Done.");

        read_parameters();
        execute_motions();
      }

    private:
      ros::NodeHandle nh_;
      std::vector< std::string > controller_specs_;
      std::vector< std::map<std::string, double> > convergence_thresholds_;
      actionlib::SimpleActionClient<giskard_msgs::WholeBodyAction> client_;

      void read_parameters()
      {
        std::vector<std::string> controller_filenames=
          readParam< std::vector<std::string> >(nh_, "controller_filenames");
        std::vector<std::string> threshold_filenames=
          readParam< std::vector<std::string> >(nh_, "threshold_filenames");
        controller_specs_ = read_ros_files(controller_filenames, "giskard_examples");
        std::vector<std::string> threshold_files = read_ros_files(threshold_filenames, "giskard_examples");
        for (size_t i=0; i<threshold_files.size(); ++i)
          convergence_thresholds_.push_back(parse_threshold_file(threshold_files[i]));
      }

      void execute_motions()
      {
        if (controller_specs_.size() != convergence_thresholds_.size())
          throw std::runtime_error("Number of controller specifications and convergence thresholds do not match.");

        for (size_t i=0; i<controller_specs_.size(); ++i)
        {
          giskard_msgs::WholeBodyGoal goal;
          goal.command.type = giskard_msgs::WholeBodyCommand::YAML_CONTROLLER;
          goal.command.yaml_spec = controller_specs_[i];
          goal.command.convergence_thresholds = to_msg(convergence_thresholds_[i]);
          ros::Time start_time = ros::Time::now();
          client_.sendGoal(goal);
          if (client_.waitForResult(ros::Duration(10)))
          {
            ros::Duration exec_time = ros::Time::now() - start_time;

            ROS_INFO("Action finished after %fs: %s", exec_time.toSec(), client_.getState().toString().c_str());
          }
          else
          {
            client_.cancelGoal();
            ROS_INFO("Action timed out.");
          }
        }

      }
  };

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "yaml_executive");
  ros::NodeHandle nh("~");

  giskard_examples::YAMLExecutive executive(nh, "controller_action_server/move");

  try
  {
    executive.run();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
  }

  return 0;
}
