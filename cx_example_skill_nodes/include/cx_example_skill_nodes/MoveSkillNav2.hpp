/***************************************************************************
 *  MoveSkillNav2.hpp
 *
 *  Created: 20 September 2021
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

#ifndef CX_EXAMPLE_SKILL_NODES__MOVESKILLNAV2_HPP
#define CX_EXAMPLE_SKILL_NODES__MOVESKILLNAV2_HPP

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

#include "lifecycle_msgs/msg/state.hpp"

#include "cx_skill_execution/SkillExecution.hpp"

#include "cx_msgs/msg/skill_action_exec_info.hpp"
#include "cx_msgs/msg/skill_execution.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace cx {
using namespace std::chrono_literals;

class MoveSkillNav2 : public cx::SkillExecution {

  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using NavGoalHandle =
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  using NavFeedback =
      const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>;

public:

  MoveSkillNav2(const std::string &id,
                const std::chrono::nanoseconds &rate);

  CallbackReturn on_activate(const rclcpp_lifecycle::State &state);

  void perform_execution() override;

private:
  std::map<std::string, geometry_msgs::msg::PoseStamped> waypoints_map_;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
      navigation_to_pose_client_;
  std::shared_future<NavGoalHandle::SharedPtr> future_navigation_goal_handle_;
  NavGoalHandle::SharedPtr navigation_goal_handle_;

  geometry_msgs::msg::Pose current_pos_;
  geometry_msgs::msg::PoseStamped goal_pos_;
  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;

  double move_distance_;
};
} // namespace cx

#endif // !CX_EXAMPLE_SKILL_NODES__MOVESKILLNAV2_HPP
