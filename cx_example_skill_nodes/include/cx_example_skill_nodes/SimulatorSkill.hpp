/***************************************************************************
 *  SimulatorSkill.hpp
 *
 *  Created: 04 August 2023
 *  Copyright  2023 Daniel Swoboda
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

#ifndef CX_EXAMPLE_SKILL_NODES__SIMULATORSKILL_HPP
#define CX_EXAMPLE_SKILL_NODES__SIMULATORSKILL_HPP

#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <regex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "cx_skill_execution/SkillExecution.hpp"

#include "cx_msgs/msg/skill_action_execinfo.hpp"
#include "cx_msgs/msg/skill_execution.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include <pyrobosim_msgs/msg/robot_state.hpp>
#include <pyrobosim_msgs/msg/task_action.hpp>

namespace cx {
using namespace std::chrono_literals;

class SimulatorSkill : public cx::SkillExecution {

  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

public:
  SimulatorSkill(const std::string &id, const std::string &action_name,
                 const std::chrono::nanoseconds &rate);

  CallbackReturn on_activate(const rclcpp_lifecycle::State &state);

  void perform_execution() override;
  void messageCallback(const pyrobosim_msgs::msg::RobotState::SharedPtr msg);

  void execute_navigate();
  void execute_sample_rock();
  void execute_sample_soil();
  void execute_drop();
  void execute_pick(std::string object_name);
  int counter_;

  bool sent_command = false;
  bool busy = false;
  bool executing = false;
  bool has_sub_action = false;
  std::string last_target = "";

  rclcpp::Subscription<pyrobosim_msgs::msg::RobotState>::SharedPtr
      subscription_;
};
} // namespace cx
#endif // !CX_EXAMPLE_SKILL_NODES__SIMULATORSKILL_HPP
