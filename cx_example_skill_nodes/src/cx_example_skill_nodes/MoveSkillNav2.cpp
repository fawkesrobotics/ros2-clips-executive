/***************************************************************************
 *  MoveSkillNav2.cpp
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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "cx_example_skill_nodes/MoveSkillNav2.hpp"

#include "lifecycle_msgs/msg/state.hpp"

#include "cx_skill_execution/SkillExecution.hpp"

#include "cx_msgs/msg/skill_action_execinfo.hpp"
#include "cx_msgs/msg/skill_execution.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace cx {

using namespace std::chrono_literals;
using namespace std::placeholders;
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using NavGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
using NavFeedback =
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>;

MoveSkillNav2::MoveSkillNav2(const std::string &id,
                             const std::string &action_name,
                             const std::chrono::nanoseconds &rate)
    : SkillExecution(id, action_name, rate) {
  geometry_msgs::msg::PoseStamped wp;
  wp.header.frame_id = "map";
  wp.header.stamp = now();
  wp.pose.position.x = 1.76;
  wp.pose.position.y = 0.68;
  wp.pose.position.z = 0.0;
  wp.pose.orientation.x = 0.0;
  wp.pose.orientation.y = 0.0;
  wp.pose.orientation.z = 0.0;
  wp.pose.orientation.w = 1.0;
  waypoints_map_["wp1"] = wp;

  wp.pose.position.x = -1.58;
  wp.pose.position.y = 1.41;
  waypoints_map_["wp2"] = wp;

  wp.pose.position.x = 1.07;
  wp.pose.position.y = -1.72;
  waypoints_map_["wp3"] = wp;

  wp.pose.position.x = 0.63;
  wp.pose.position.y = 1.74;
  waypoints_map_["wp4"] = wp;

  wp.pose.position.x = 0.63;
  wp.pose.position.y = 1.74;
  waypoints_map_["wp5"] = wp;

  wp.pose.position.x = -2.57;
  wp.pose.position.y = 0.02;
  waypoints_map_["wp6"] = wp;

  wp.pose.position.x = -2.0;
  wp.pose.position.y = -0.4;
  waypoints_map_["wp_init"] = wp;

  wp.pose.position.x = -2.0;
  wp.pose.position.y = -0.4;
  waypoints_map_["wp_final"] = wp;
}

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn
MoveSkillNav2::on_activate(const rclcpp_lifecycle::State &state) {
  std::cerr << "MoveSkillNav2::on_activate" << std::endl;
  send_feedback(0.0, "Move starting");

  navigation_to_pose_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          shared_from_this(), "navigate_to_pose");

  while (navigation_to_pose_client_->wait_for_action_server(
      std::chrono::seconds(4))) {

    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Timed out waiting for server availability");
      return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(get_logger(), "Waiting for navigation action server...");
  }

  if (!navigation_to_pose_client_->action_server_is_ready()) {
    finish_execution(false, 0.0, "Action server unavailable!");
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "Navigation action server ready");

  auto wp_to_navigate =
      action_parameters_[2]; // The goal wp is in the 2nd argument
  RCLCPP_INFO(get_logger(), "Start navigation to [%s]", wp_to_navigate.c_str());

  goal_pos_ = waypoints_map_[wp_to_navigate];
  navigation_goal_.pose = goal_pos_;

  move_distance_ =
      sqrt((goal_pos_.pose.position.x - current_pos_.position.x) *
               (goal_pos_.pose.position.x - current_pos_.position.x) +
           (goal_pos_.pose.position.y - current_pos_.position.y) *
               (goal_pos_.pose.position.y - current_pos_.position.y));

  auto send_goal_options = rclcpp_action::Client<
      nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  send_goal_options.feedback_callback = [this](NavGoalHandle::SharedPtr,
                                               NavFeedback feedback) {
    send_feedback(1.0 - (feedback->distance_remaining / move_distance_),
                  "Move running");
  };

  send_goal_options.result_callback = [this](auto) {
    finish_execution(true, 1.0, "Move completed");
  };

  future_navigation_goal_handle_ = navigation_to_pose_client_->async_send_goal(
      navigation_goal_, send_goal_options);

  return SkillExecution::on_activate(state);
}

void MoveSkillNav2::perform_execution() {
  RCLCPP_INFO_STREAM(get_logger(), "Executing [" << action_name_ << "]");
  for (const auto &param : action_parameters_) {
    RCLCPP_INFO_STREAM(get_logger(), "\t[" << param << "]");
  }
}

} // namespace cx
