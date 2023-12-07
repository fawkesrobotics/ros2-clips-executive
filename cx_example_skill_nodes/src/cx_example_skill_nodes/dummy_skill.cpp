/***************************************************************************
 *  DummySkill.cpp
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

#include "cx_example_skill_nodes/dummy_skill.hpp"

#include "lifecycle_msgs/msg/state.hpp"

namespace cx {

using namespace std::chrono_literals;

DummySkill::DummySkill(const std::string &id,
                               const std::chrono::nanoseconds &rate)
    : SkillExecution(id, rate) {
      executor_id_ = "dummy_skiller";
    }
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn
DummySkill::on_activate(const rclcpp_lifecycle::State &state) {

  declare_parameter("exec_times." + action_name_ + ".time",5.0);
  get_parameter("exec_times." + action_name_ + ".time",duration_);
	if(timer_) {
		timer_->cancel();
	}
	auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<float>(duration_));
	  auto callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);


	timer_ = rclcpp::create_timer(this, get_clock(), period_ns, [this] () {
	  if (timer_ && get_current_state().id() ==
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
		RCLCPP_INFO(get_logger(), "Lambda function called after 7 seconds!");
      finish_execution(true, 1.0, "completed");
		timer_->cancel();
	   }
	  
	},
							   nullptr
	);
  RCLCPP_INFO_STREAM(get_logger(), "Dummy Execution of " << action_name_ << " takes " << duration_ << " seconds");
  for (const auto &param : action_parameters_) {
    RCLCPP_INFO_STREAM(get_logger(), "\t[" << param << "]");
  }
  return SkillExecution::on_activate(state);

}
    // Override the finish_execution function
void DummySkill::finish_execution(bool success, float progress,
                        const std::string &status) {
        // Call the base class implementation
	    timer_->cancel();
        SkillExecution::finish_execution(success, progress, status);
    }
void DummySkill::perform_execution() {
	if(!timer_) {
	on_activate(get_current_state());
	}
}

} // namespace cx
