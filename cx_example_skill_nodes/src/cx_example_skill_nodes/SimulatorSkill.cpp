/***************************************************************************
 *  SimulatorSkill.cpp
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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "cx_example_skill_nodes/SimulatorSkill.hpp"

#include "lifecycle_msgs/msg/state.hpp"
namespace cx {

using namespace std::chrono_literals;
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

SimulatorSkill::SimulatorSkill(const std::string &id,
                               const std::string &action_name,
                               const std::chrono::nanoseconds &rate)
    : SkillExecution(id, action_name, rate) {

  subscription_ = create_subscription<pyrobosim_msgs::msg::RobotState>(
      "/robot/robot_state", 10,
      std::bind(&SimulatorSkill::messageCallback, this, std::placeholders::_1));

  RCLCPP_INFO_STREAM(get_logger(), "Starting SimulatorSkill Node!");
}

CallbackReturn
SimulatorSkill::on_activate(const rclcpp_lifecycle::State &state) {
  std::cerr << "SimulatorSkill::on_activate" << std::endl;
  counter_ = 0;

  return SkillExecution::on_activate(state);
}

void SimulatorSkill::perform_execution() {
  if (!sent_command && !busy && !executing) {
    RCLCPP_INFO_STREAM(get_logger(), "Executing [ " << action_name_ << " ]");
    for (const auto &param : action_parameters_) {
      RCLCPP_INFO_STREAM(get_logger(), "\t[" << param << "]");
    }
    if (action_name_ == "navigate") {
      execute_navigate();
    }
    if (action_name_ == "sample_soil") {
      execute_sample_soil();
      has_sub_action = true;
    }
    if (action_name_ == "drop") {
      execute_drop();
    }
    if (action_name_ == "sample_rock") {
      execute_sample_rock();
      has_sub_action = true;
    }
  }

  if (executing) {
    send_feedback(0.5, "running");
  }
  if (executing && !busy) {

    RCLCPP_INFO_STREAM(get_logger(), "Status indicates finished");
    executing = false;
    sent_command = false;

    if (has_sub_action &&
        (action_name_ == "sample_soil" || action_name_ == "sample_rock")) {
      RCLCPP_INFO_STREAM(get_logger(), "Executing [ "
                                           << action_name_
                                           << " ] picking sub-action");
      if (action_name_ == "sample_soil") {
        execute_pick("soil");
      } else {
        execute_pick("rock");
      }
      has_sub_action = false;
    } else {
      finish_execution(true, 1.0, "completed");
      RCLCPP_INFO_STREAM(get_logger(), "Finished [ " << action_name_ << " ]");
    }
  }
}

void SimulatorSkill::execute_navigate() {
  pyrobosim_msgs::msg::TaskAction message;
  message.type = "navigate";
  message.target_location = action_parameters_[2];
  message.robot = "robot";
  RCLCPP_INFO_STREAM(get_logger(),
                     "Sending message navigate to " << action_parameters_[2]);

  auto task_publisher = create_publisher<pyrobosim_msgs::msg::TaskAction>(
      "commanded_action", rclcpp::QoS(100).reliable());
  task_publisher->on_activate();
  task_publisher->publish(message);

  sent_command = true;
}

void SimulatorSkill::execute_pick(std::string object_name) {
  pyrobosim_msgs::msg::TaskAction message;
  message.type = "pick";
  message.object = action_parameters_[2] + "_" + object_name + "_sample";
  message.robot = "robot";
  message.room = action_parameters_[2];
  RCLCPP_INFO_STREAM(get_logger(), "Sending message pick for "
                                       << action_parameters_[2] + "_" +
                                              object_name + "_sample");

  auto task_publisher = create_publisher<pyrobosim_msgs::msg::TaskAction>(
      "commanded_action", rclcpp::QoS(100).reliable());
  task_publisher->on_activate();
  task_publisher->publish(message);

  // handle immediatly as we do not get a status update for this
  executing = true;
  busy = false;
  sent_command = true;
}

void SimulatorSkill::execute_drop() {
  pyrobosim_msgs::msg::TaskAction message;
  message.type = "place";
  message.robot = "robot";
  RCLCPP_INFO_STREAM(get_logger(), "Sending message place");

  auto task_publisher = create_publisher<pyrobosim_msgs::msg::TaskAction>(
      "commanded_action", rclcpp::QoS(100).reliable());
  task_publisher->on_activate();
  task_publisher->publish(message);

  // handle immediatly as we do not get a status update for this
  executing = true;
  busy = false;
  sent_command = true;
}

void SimulatorSkill::execute_sample_soil() {
  pyrobosim_msgs::msg::TaskAction message;
  message.type = "navigate";
  message.target_location = action_parameters_[2] + "_soil_sample";
  message.robot = "robot";
  message.room = action_parameters_[2];
  RCLCPP_INFO_STREAM(get_logger(), "Sending message navigate to "
                                       << action_parameters_[2] + "_soil");

  auto task_publisher = create_publisher<pyrobosim_msgs::msg::TaskAction>(
      "commanded_action", rclcpp::QoS(100).reliable());
  task_publisher->on_activate();
  task_publisher->publish(message);

  sent_command = true;
}

void SimulatorSkill::execute_sample_rock() {
  pyrobosim_msgs::msg::TaskAction message;
  message.type = "navigate";
  message.target_location = action_parameters_[2] + "_rock";
  message.robot = "robot";
  message.room = action_parameters_[2];
  RCLCPP_INFO_STREAM(get_logger(), "Sending message navigate to "
                                       << action_parameters_[2] + "_rock");

  auto task_publisher = create_publisher<pyrobosim_msgs::msg::TaskAction>(
      "commanded_action", rclcpp::QoS(100).reliable());
  task_publisher->on_activate();
  task_publisher->publish(message);

  sent_command = true;
}

void SimulatorSkill::messageCallback(
    const pyrobosim_msgs::msg::RobotState::SharedPtr msg) {
  if (msg->executing_action == 1) {
    busy = true;
  }
  if (msg->executing_action == 0) {
    busy = false;
    // RCLCPP_INFO(get_logger(), "AAAA CURR STATE NON_BUSY!!");
  }
  if (busy && sent_command) {
    executing = true;
  }
}

} // namespace cx
