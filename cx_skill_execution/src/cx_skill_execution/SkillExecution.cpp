// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  SkillExecution.cpp
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
  https://github.com/IntelligentRoboticsLabs/ros2_planning_system/blob/master/plansys2_executor/src/plansys2_executor/ActionExecutorClient.cpp
*/

/*
  Classes implmenting a ceratin action should
  inherit from the class and implement their
  individual behaviour.
*/

#include <memory>
#include <string>

#include "cx_skill_execution/SkillExecution.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace cx {
using namespace std::chrono_literals;
using SkillExecutionMsg = cx_msgs::msg::SkillExecution;

SkillExecution::SkillExecution(const std::string &node_name,
                               const std::chrono::nanoseconds &exec_rate,
                               const rclcpp::NodeOptions &options,
                               const std::string &namespace_)
    : rclcpp_lifecycle::LifecycleNode(node_name, namespace_, options),
      exec_rate_(exec_rate), robot_id_(""), executor_id_(node_name) {

  RCLCPP_INFO(get_logger(), "Initialising executor %s for robot %s",
              node_name.c_str(), robot_id_.c_str());

  declare_parameter("robot_id", robot_id_);
  robot_id_ = get_parameter("robot_id").as_string();
}

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using std::placeholders::_1;

CallbackReturn
SkillExecution::on_configure(const rclcpp_lifecycle::State &state) {
  (void)state;
  skill_board_pub = create_publisher<SkillExecutionMsg>(
      "/skill_board", rclcpp::QoS(100).reliable());
  skill_board_sub = create_subscription<SkillExecutionMsg>(
      "/skill_board", rclcpp::QoS(100).reliable(),
      std::bind(&SkillExecution::skill_board_cb, this, _1));

  skill_board_pub->on_activate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
SkillExecution::on_activate(const rclcpp_lifecycle::State &state) {
  (void)state;
  execution_heartbeat_ = create_wall_timer(
      exec_rate_, std::bind(&SkillExecution::perform_execution, this));
  perform_execution();

  return CallbackReturn::SUCCESS;
}
CallbackReturn
SkillExecution::on_deactivate(const rclcpp_lifecycle::State &state) {
  (void)state;
  execution_heartbeat_ = nullptr;
  return CallbackReturn::SUCCESS;
}

void SkillExecution::skill_board_cb(
    const cx_msgs::msg::SkillExecution::SharedPtr msg) {

  // Check the type of the published message
  /*
    Important types for the executor:
    1. Request - coming from clips feature/clips directly
    2. Confirm - give confirmation that the skill will be executed
    3. Reject - request to stop skill execution
    4. Cancel - cancel skill execution
  */

  switch (msg->type) {
  case SkillExecutionMsg::RESPONSE:
  case SkillExecutionMsg::FEEDBACK:
  case SkillExecutionMsg::FINISH:
    break;
  case SkillExecutionMsg::REQUEST:
    if (get_current_state().id() ==
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE &&
        !commited_to_skill_ && robot_id_ == msg->robot_id &&
        executor_id_ == msg->executor_id) {
      commited_to_skill_ = true;
      send_response(msg);
    }
    break;
  case SkillExecutionMsg::CONFIRM:
    if (get_current_state().id() ==
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE &&
        commited_to_skill_ && msg->robot_id == robot_id_ &&
        msg->executor_id == executor_id_) {
      action_name_ = msg->action;
      action_parameters_ = msg->action_parameters;
      mapped_action_ = msg->mapped_action;
      // Transition to active state, so the implemented function can be executed
      trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
      commited_to_skill_ = false;
    }
    break;
  case SkillExecutionMsg::REJECT:
    if (msg->node_id == get_name() && msg->robot_id == robot_id_ &&
        msg->executor_id == executor_id_) {
      commited_to_skill_ = false;
    }
    break;
  case SkillExecutionMsg::CANCEL:
    if (get_current_state().id() ==
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE &&
        msg->node_id == get_name() && msg->executor_id == executor_id_) {
      finish_execution(false, 0.0,
                       /*expected from the exec master*/ "CANCELLED");
    }
    break;
  default:
    RCLCPP_ERROR(get_logger(), "Message type: %d not recognized!", msg->type);
    break;
  }
}

void SkillExecution::send_response(
    const cx_msgs::msg::SkillExecution::SharedPtr msg) {
  SkillExecutionMsg rsp = *msg;
  rsp.type = SkillExecutionMsg::RESPONSE;
  rsp.node_id = get_name();
  skill_board_pub->publish(rsp);
}

void SkillExecution::send_feedback(float progress, const std::string &status) {
  SkillExecutionMsg msg;
  msg.type = SkillExecutionMsg::FEEDBACK;
  msg.node_id = get_name();
  msg.action = action_name_;
  msg.action_parameters = action_parameters_;
  msg.mapped_action = mapped_action_;
  msg.progress = progress;
  msg.status = status;
  msg.robot_id = robot_id_;
  msg.executor_id = executor_id_;

  skill_board_pub->publish(msg);
}

void SkillExecution::finish_execution(bool success, float progress,
                                      const std::string &status) {
  if (get_current_state().id() ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  }

  SkillExecutionMsg msg;
  msg.type = SkillExecutionMsg::FINISH;
  msg.node_id = get_name();
  msg.action = action_name_;
  msg.action_parameters = action_parameters_;
  msg.mapped_action = mapped_action_;
  msg.progress = progress;
  msg.status = status;
  msg.success = success;
  msg.robot_id = robot_id_;
  msg.executor_id = executor_id_;

  skill_board_pub->publish(msg);
}

} // namespace cx
