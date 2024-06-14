// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  SkillExecutionMaster.hpp
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
  https://github.com/IntelligentRoboticsLabs/ros2_planning_system/blob/master/plansys2_executor/include/plansys2_executor/ActionExecutor.hpp
*/

#ifndef CX_SKILL_EXECUTION__SKILLEXECUTIONMASTER_HPP
#define CX_SKILL_EXECUTION__SKILLEXECUTIONMASTER_HPP

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "cx_msgs/msg/skill_action_exec_info.hpp"
#include "cx_msgs/msg/skill_execution.hpp"
#include "cx_utils/LockSharedPtr.hpp"
#include <clipsmm.h>

namespace cx {

class SkillExecutionMaster : public rclcpp::Node {

public:
  enum ExecState { IDLE, RUNNING, CANCELLED, SUCCESS, FAILURE };
  SkillExecutionMaster(
      const std::string &node_name, const std::string &skill_id,
      const std::string &action_name, const std::string &action_parameters,
      const std::string &mapped_action, const std::string &robot_id,
      const std::string &executor_id,
      cx::LockSharedPtr<CLIPS::Environment> &clips,
      const std::string &namespace_ = "",
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  void request_skill_execution();
  void cancel_execution();
  void check_idle_time();

  ExecState get_exec_status() const;

  cx_msgs::msg::SkillActionExecInfo get_exec_info() const {
    return exec_info_;
  };

  std::string get_action_name() const { return action_name_; }
  std::vector<std::string> get_action_params() const {
    return action_parameters_;
  }

protected:
  void skill_board_cb(const cx_msgs::msg::SkillExecution::SharedPtr msg);
  void confirm_executor(const std::string &node_id);
  void reject_skill_execution(const std::string &node_id);
  std::vector<std::string> extract_parameters(const std::string &action_params);

protected:
  const std::string skill_id_;
  const std::string action_name_;
  const std::string action_mapping_;
  const std::string mapped_action_;
  const std::string string_action_parameters_;
  const std::string robot_id_;
  const std::string executor_id_;
  std::vector<std::string> action_parameters_;
  std::string node_id_;

  cx::LockSharedPtr<CLIPS::Environment> &clips_;
  ExecState state_;
  cx_msgs::msg::SkillActionExecInfo exec_info_{};
  std::string feedback_{};
  float progress_{};

  rclcpp::Time current_state_time_;
  rclcpp::Time exec_start_;

  rclcpp::Publisher<cx_msgs::msg::SkillExecution>::SharedPtr skill_board_pub;
  rclcpp::Subscription<cx_msgs::msg::SkillExecution>::SharedPtr skill_board_sub;
};
} // namespace cx
#endif // !CX_SKILL_EXECUTION__SKILLEXECUTIONMASTER_HPP
