/***************************************************************************
 *  DummySkill.hpp
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

#ifndef CX_EXAMPLE_SKILL_NODES__DUMMYMOVESKILL_HPP
#define CX_EXAMPLE_SKILL_NODES__DUMMYMOVESKILL_HPP

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

#include "cx_msgs/msg/skill_action_exec_info.hpp"
#include "cx_msgs/msg/skill_execution.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace cx {
using namespace std::chrono_literals;

class DummySkill : public cx::SkillExecution {

  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

public:
  DummySkill(const std::string &id,
                 const std::chrono::nanoseconds &rate);

  CallbackReturn on_activate(const rclcpp_lifecycle::State &state);

  void perform_execution() override;

  int counter_;
};
} // namespace cx
#endif // !CX_EXAMPLE_SKILL_NODES__DUMMYMOVESKILL_HPP
