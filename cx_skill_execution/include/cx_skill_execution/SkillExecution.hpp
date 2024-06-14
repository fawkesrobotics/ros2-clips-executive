// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  SkillExecution.hpp
 *
 *  Created: 16 September 2021
 *  Copyright  2021  Ivaylo Doychev
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

/*
  Inspired by:
  Plansys2, Intelligent Robotics Lab
  https://github.com/IntelligentRoboticsLabs/ros2_planning_system/blob/master/plansys2_executor/include/plansys2_executor/ActionExecutorClient.hpp
*/

/*
  Classes implmenting a certain action should
  inherit from the class and implement their
  individual behaviour.
*/

#ifndef CX_SKILLS_EXECUTION__SKILLEXECUTION_HPP
#define CX_SKILLS_EXECUTION__SKILLEXECUTION_HPP

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "cx_msgs/msg/skill_execution.hpp"

namespace cx {

class SkillExecution : public rclcpp_lifecycle::LifecycleNode {

public:
  SkillExecution(const std::string &node_name,
                 const std::chrono::nanoseconds &pub_rate,
                 const rclcpp::NodeOptions &options = rclcpp::NodeOptions(),
                 const std::string &namespace_ = "");

protected:
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state);
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);

  virtual void perform_execution() {};

  void skill_board_cb(const cx_msgs::msg::SkillExecution::SharedPtr msg);

  void send_response(const cx_msgs::msg::SkillExecution::SharedPtr msg);
  void send_feedback(float progress, const std::string &status);
  virtual void finish_execution(bool success, float progress,
                                const std::string &status);

  std::string action_name_;
  std::vector<std::string> action_parameters_;
  std::string mapped_action_;
  const std::chrono::nanoseconds exec_rate_;
  bool commited_to_skill_;
  std::string robot_id_;
  std::string executor_id_;

  rclcpp_lifecycle::LifecyclePublisher<cx_msgs::msg::SkillExecution>::SharedPtr
      skill_board_pub;
  rclcpp::Subscription<cx_msgs::msg::SkillExecution>::SharedPtr skill_board_sub;
  // Creates wall timer for the implemented working function
  rclcpp::TimerBase::SharedPtr execution_heartbeat_;
};
} // namespace cx

#endif // !CX_SKILLS_EXECUTION__SKILLEXECUTION_HPP
