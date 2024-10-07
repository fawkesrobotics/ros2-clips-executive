// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  SkillExecutionMaster.cpp
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
  Inspired by:
  Plansys2, Intelligent Robotics Lab
  https://github.com/IntelligentRoboticsLabs/ros2_planning_system/blob/master/plansys2_executor/src/plansys2_executor/ActionExecutor.cpp
*/

#include <format>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "cx_skill_execution/SkillExecutionMaster.hpp"

#include "cx_msgs/msg/skill_action_exec_info.hpp"
#include "cx_msgs/msg/skill_execution.hpp"

namespace cx {
using std::placeholders::_1;
using namespace std::chrono_literals;
using SkillExecutionMsg = cx_msgs::msg::SkillExecution;
using SkillActionExecInfo = cx_msgs::msg::SkillActionExecInfo;

SkillExecutionMaster::SkillExecutionMaster(
    const std::string &node_name, const std::string &skill_id,
    const std::string &action_name, const std::string &mapped_action,
    const std::string &action_parameters, const std::string &robot_id,
    const std::string &executor_id,
    cx::LockSharedPtr<clips::Environment> &clips, const std::string &ns,
    const rclcpp::NodeOptions &options)
    : rclcpp::Node(node_name, ns, options), skill_id_(skill_id),
      action_name_(action_name), mapped_action_(mapped_action),
      string_action_parameters_(action_parameters), robot_id_(robot_id),
      executor_id_(executor_id), clips_(clips) {

  skill_board_pub = create_publisher<SkillExecutionMsg>(
      "/skill_board", rclcpp::QoS(100).reliable());
  skill_board_sub = create_subscription<SkillExecutionMsg>(
      "/skill_board", rclcpp::QoS(100).reliable(),
      std::bind(&SkillExecutionMaster::skill_board_cb, this, _1));

  action_parameters_ = extract_parameters(action_parameters);
  state_ = IDLE;
  current_state_time_ = now();
  // Relevant for skill exec feature
  exec_info_.status_stamp = current_state_time_;
  exec_info_.start_stamp = current_state_time_;
  exec_info_.status = SkillActionExecInfo::S_IDLE;
  exec_info_.string_status = "S_IDLE";
  exec_info_.skill_id = skill_id;
  exec_info_.action = action_name;
  exec_info_.action_parameters = action_parameters_;
  exec_info_.mapped_action = mapped_action;
  exec_info_.robot_id = robot_id_;
  exec_info_.executor_id = executor_id_;
}

void SkillExecutionMaster::skill_board_cb(
    const cx_msgs::msg::SkillExecution::SharedPtr msg) {

  // Check the type of the published message
  /*
    Important types for the execution master:
    1. RESPONSE - coming from the executor after sending init req
    2. FEEDBACK - action execution feedback from executor
    3. FINISH - sent from executor after the succesful/failed execution
  */

  switch (msg->type) {
  case SkillExecutionMsg::REQUEST:
  case SkillExecutionMsg::CONFIRM:
  case SkillExecutionMsg::REJECT:
  case SkillExecutionMsg::CANCEL:
    break;
  case SkillExecutionMsg::RESPONSE:
    if (msg->action_parameters == action_parameters_ &&
        msg->action == action_name_ && msg->robot_id == robot_id_ &&
        msg->executor_id == executor_id_) {
      confirm_executor(msg->executor_id);
      node_id_ = msg->node_id;
      state_ = RUNNING;
      exec_info_.status = SkillActionExecInfo::S_RUNNING;
      exec_info_.string_status = "S_RUNNING";
      exec_start_ = now();
      exec_info_.start_stamp = exec_start_;
      current_state_time_ = now();
      exec_info_.status_stamp = current_state_time_;
    }
    break;
  case SkillExecutionMsg::FEEDBACK:
    if (state_ != RUNNING || msg->action_parameters != action_parameters_ ||
        msg->action != action_name_ || msg->node_id != node_id_ ||
        msg->robot_id != robot_id_ || msg->executor_id != executor_id_) {
      return;
    }
    feedback_ = msg->status;
    progress_ = msg->progress;
    current_state_time_ = now();
    break;
  case SkillExecutionMsg::FINISH:
    if (msg->action_parameters == action_parameters_ &&
        msg->action == action_name_ && msg->node_id == node_id_ &&
        msg->robot_id == robot_id_ && msg->executor_id == executor_id_) {
      if (msg->status == "CANCELLED") {
        state_ = CANCELLED;
        exec_info_.status = SkillActionExecInfo::S_FAILED;
        exec_info_.string_status = "S_FAILED";
        exec_info_.error_msg = msg->status;
      } else {
        state_ = msg->success ? SUCCESS : FAILURE;
        exec_info_.status = msg->success ? SkillActionExecInfo::S_FINAL
                                         : SkillActionExecInfo::S_FAILED;
        exec_info_.string_status = msg->success ? "S_FINAL" : "S_FAILED";
        exec_info_.error_msg = msg->success ? "" : msg->status;
      }
      feedback_ = msg->status;
      progress_ = msg->progress;

      current_state_time_ = now();

      skill_board_pub = nullptr;
      skill_board_sub = nullptr;
    }
    break;
  default:
    RCLCPP_ERROR(
        get_logger(), "Message type: %d not recognized for action (%s %s)!",
        msg->type, action_name_.c_str(), string_action_parameters_.c_str());
    break;
  }

  // update the clips environment

  std::lock_guard<std::mutex> guard(*(clips_.get_mutex_instance()));
  clips::AssertString(clips_.get_obj().get(),
                      std::format("(skill-feedback (skill-id {}) (robot "
                                  "\"{}\") (executor \"{}\") (status "
                                  "{}) (error "
                                  "\"{}\") (time (now)))",
                                  exec_info_.skill_id.c_str(),
                                  exec_info_.robot_id.c_str(),
                                  exec_info_.executor_id.c_str(),
                                  exec_info_.string_status.c_str(),
                                  exec_info_.error_msg.c_str())
                          .c_str());
}

void SkillExecutionMaster::request_skill_execution() {
  SkillExecutionMsg msg;
  msg.type = SkillExecutionMsg::REQUEST;
  msg.node_id = get_name();
  msg.action = action_name_;
  msg.action_parameters = action_parameters_;
  msg.mapped_action = mapped_action_;
  msg.robot_id = robot_id_;
  msg.executor_id = executor_id_;

  skill_board_pub->publish(msg);
  RCLCPP_WARN(get_logger(), "Sent execution request for action %s!",
              action_name_.c_str());
}

void SkillExecutionMaster::confirm_executor(const std::string &node_id) {
  SkillExecutionMsg msg;
  msg.type = SkillExecutionMsg::CONFIRM;
  msg.node_id = node_id;
  msg.action = action_name_;
  msg.action_parameters = action_parameters_;
  msg.mapped_action = mapped_action_;
  msg.robot_id = robot_id_;
  msg.executor_id = executor_id_;

  skill_board_pub->publish(msg);
}

void SkillExecutionMaster::cancel_execution() {
  SkillExecutionMsg msg;
  msg.type = SkillExecutionMsg::CANCEL;
  msg.node_id = node_id_;
  msg.action = action_name_;
  msg.action_parameters = action_parameters_;
  msg.robot_id = robot_id_;
  msg.executor_id = executor_id_;

  skill_board_pub->publish(msg);
}

void SkillExecutionMaster::check_idle_time() {
  if (state_ == IDLE && (now() - current_state_time_).seconds() > 4.0) {
    state_ = FAILURE;
    exec_info_.status = SkillActionExecInfo::S_FAILED;
    exec_info_.string_status = "S_FAILED";
    exec_info_.error_msg =
        "Timed out (4s) waiting for response from execution node for robot " +
        robot_id_ + " and executor " + executor_id_ + " and skill (" +
        action_name_ + " " + string_action_parameters_ + ")";
  }
}

std::vector<std::string>
SkillExecutionMaster::extract_parameters(const std::string &action_params) {
  std::vector<std::string> params_vec;
  size_t start_pos = 0, end_pos = 0;
  while (end_pos != std::string::npos) {
    end_pos = action_params.find(" ", start_pos);
    auto cur_param = action_params.substr(
        start_pos, (end_pos == std::string::npos) ? std::string::npos
                                                  : end_pos - start_pos);
    params_vec.push_back(cur_param);
    start_pos =
        ((end_pos > (std::string::npos - 1)) ? std::string::npos : end_pos + 1);
  }
  return params_vec;
}

SkillExecutionMaster::ExecState SkillExecutionMaster::get_exec_status() const {
  return state_;
}

} // namespace cx
