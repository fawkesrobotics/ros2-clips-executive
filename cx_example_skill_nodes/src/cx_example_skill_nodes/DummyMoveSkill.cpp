/***************************************************************************
 *  DummyMoveSkill.cpp
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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "cx_example_skill_nodes/DummyMoveSkill.hpp"

#include "lifecycle_msgs/msg/state.hpp"

namespace cx {

using namespace std::chrono_literals;

DummyMoveSkill::DummyMoveSkill(const std::string &id,
                               const std::chrono::nanoseconds &rate)
    : SkillExecution(id, rate) {
      executor_id_ = "dummy_skiller";
    }
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn
DummyMoveSkill::on_activate(const rclcpp_lifecycle::State &state) {
  std::cerr << "DummyMoveSkill::on_activate" << std::endl;
  counter_ = 0;

  return SkillExecution::on_activate(state);
}

void DummyMoveSkill::perform_execution() {
  RCLCPP_INFO_STREAM(get_logger(), "Dummy Execution of ");
  RCLCPP_INFO_STREAM(get_logger(), action_name_.c_str());
  for (const auto &param : action_parameters_) {
    RCLCPP_INFO_STREAM(get_logger(), "\t[" << param << "]");
  }

  if (counter_++ > 3) {
    finish_execution(true, 1.0, "completed");
  } else {
    send_feedback(counter_ * 0.0, "running");
  }
}

} // namespace cx
