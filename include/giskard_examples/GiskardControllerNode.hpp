/*
* Copyright (C) 2015, 2016 Mihai Pomarlan <blandc@cs.uni-bremen.de>
* Jannik Buckelo <jannikbu@cs.uni-bremen.de>,
* Georg Bartels <georg.bartels@cs.uni-bremen.de>,*
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


#ifndef __GISKARD_CONTROLLER_NODE_HPP__
#define __GISKARD_CONTROLLER_NODE_HPP__

#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <yaml-cpp/yaml.h>
#include <giskard/giskard.hpp>

#include <giskard_msgs/SetEnable.h>
#include <giskard_msgs/Finished.h>
#include <boost/bind.hpp>

namespace gcn{

template<int goalSize, typename GoalMsgType> class GiskardControllerNode
{
public:
    typedef GiskardControllerNode<goalSize, GoalMsgType> ControllerType;
    typedef boost::shared_ptr<GoalMsgType> GoalMsgPtr;
    typedef boost::shared_ptr<GoalMsgType const> GoalMsgConstPtr;
    typedef void(*GoalParserFnPtr)(GoalMsgConstPtr const& goalMsg, std::vector<double> &goalValues);

    GiskardControllerNode(GoalParserFnPtr goalParser, std::string const& initString=std::string("~")):
        goalSize_(goalSize), nh_(initString), goalParser_(goalParser)
    {
        nh_.param("nWSR", nWSR_, 10);
        nh_.param("is_done_topic", doneTopic_, std::string("doneMovement"));
        active_ = false;
        moving_ = false;
        controller_started_ = false;
        isInitialized_ = false;
        spec_.controllable_constraints_.clear();
        spec_.hard_constraints_.clear();
        spec_.soft_constraints_.clear();
        spec_.scope_.clear();
    }

    bool isInitialized()
    {
        if(!isInitialized_)
            isInitialized_ = InitializeController();
        return isInitialized_;
    }

private:
    bool InitializeController(void)
    {
        std::string controller_description;

        if (nh_.getParam("controller_description", controller_description))
        {
            if (nh_.getParam("joint_names", joint_names_))
            {
                YAML::Node node = YAML::Load(controller_description);
                ROS_INFO("Loaded controller description.");
                spec_ = node.as< giskard::QPControllerSpec >();
                ROS_INFO("Parsed controller description.");
                controller_ = giskard::generate(spec_);
                ROS_INFO("Generated controller description.");
                state_ = Eigen::VectorXd::Zero(joint_names_.size() + goalSize_);
                ROS_INFO("Created a state.");
                controller_started_ = false;

                int maxK = spec_.controllable_constraints_.size();
                controllable_joint_names_.resize(maxK);
                for(int k = 0; k < maxK; k++)
                    controllable_joint_names_[k] = joint_names_[spec_.controllable_constraints_[k].input_number_];

                std::cout << "CONTROLLABLE JOINTS\n";
                for(int k = 0; k < maxK; k++)
                    std::cout << "\t" << controllable_joint_names_[k] << std::endl;
    
                for (std::vector<std::string>::iterator it = controllable_joint_names_.begin(); it != controllable_joint_names_.end(); ++it)
                    vel_controllers_.push_back(nh_.advertise<std_msgs::Float64>("/" + *it + "/vel_cmd", 1));
    
                enable_service_ = nh_.advertiseService("SetEnable", &ControllerType::doSetEnable, this);
                ROS_INFO("Waiting for seggoal.");
                goal_sub_ = nh_.subscribe("goal", 0, &ControllerType::goalCallback, this);
                js_sub_ = nh_.subscribe("joint_states", 0, &ControllerType::jointCallback, this);
                done_adv_ = nh_.advertise<giskard_msgs::Finished>("finished", 0);
            }
            else
            {
                ROS_ERROR("Parameter 'joint_names' not found in namespace '%s'.", nh_.getNamespace().c_str());
                return false;
            }
        }
        else
        {
            ROS_ERROR("Parameter 'controller_description' not found in namespace '%s'.", nh_.getNamespace().c_str());
            return false;
        }
        return true;
    }

    void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        if ((!controller_started_) || (!active_))
          return;

      // TODO: turn this into a map!
      // is there a more efficient way?
        for (unsigned int i=0; i < joint_names_.size(); i++)
        {
            for (unsigned int j=0; j < msg->name.size(); j++)
            {
                if (msg->name[j].compare(joint_names_[i]) == 0)
                {
                    state_[i] = msg->position[j];
                }
            }
        }

        if (controller_.update(state_, nWSR_))
        {
            Eigen::VectorXd commands = controller_.get_command();
            double commPower = 0;
            for (unsigned int i=0; i < vel_controllers_.size(); i++)
            {
                std_msgs::Float64 command;
                command.data = commands[i];
                commPower += (commands[i]*commands[i]);
                vel_controllers_[i].publish(command);
            }
            if(moving_ && (commPower < 0.001))
            {
                done_adv_.publish(giskard_msgs::Finished());
                moving_ = false;
            }
            else
                moving_ = true;
        }
        else
        {
            ROS_WARN("Update failed.");
            // TODO: remove or change to ros_debug
            std::cout << "State " << state_ << std::endl;
        }
    }
    void goalCallback(const GiskardControllerNode<goalSize, GoalMsgType>::GoalMsgConstPtr& msg)
    {
        std::vector<double> aux; aux.clear();
        goalParser_(msg, aux);
    
        int maxK = aux.size();
    
        for(int k = 0; k < maxK; k++)
            state_[joint_names_.size() + k] = aux[k];
    
        if(!active_)
            return;
    
        if (!controller_started_)
        {
            if (controller_.start(state_, nWSR_))
            {
                ROS_INFO("Controller started.");
                controller_started_ = true;
            }
            else
            {
                ROS_ERROR("Couldn't start controller.");
            }
        }
    }

    bool doSetEnable(giskard_msgs::SetEnable::Request &req, giskard_msgs::SetEnable::Response &res)
    {
        active_ = req.enable;
        return true;
    }

    int goalSize_;
    ros::NodeHandle nh_;
    GoalParserFnPtr goalParser_;
    int nWSR_;
    bool active_;
    bool moving_;
    std::string doneTopic_;
    bool isInitialized_;

    giskard::QPController controller_;
    std::vector<std::string> joint_names_;
    std::vector<std::string> controllable_joint_names_;
    std::vector<ros::Publisher> vel_controllers_;
    ros::Subscriber js_sub_;
    Eigen::VectorXd state_;
    bool controller_started_;

    ros::ServiceServer enable_service_;
    ros::Subscriber goal_sub_;
    giskard::QPControllerSpec spec_;

    ros::Publisher done_adv_;

};

}

#endif
