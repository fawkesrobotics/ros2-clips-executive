// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  cx_node.cpp
 *
 *  Created: 14 July 2021
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

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "cx_clips_env_manager/clips_env_manager.h"

std::shared_ptr<cx::CLIPSEnvManager> clips_env_manager_node;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe;

void custom_signal_handler(int /*signum*/) {
  if (clips_env_manager_node) {
    auto current_state = clips_env_manager_node->get_current_state();
    const std::chrono::seconds timeout_duration(5);

    // Check if the node is active before trying to deactivate
    if (current_state.id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      clips_env_manager_node->trigger_transition(
          lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);

      auto start_time = std::chrono::steady_clock::now();

      while (current_state.id() !=
             lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        current_state = clips_env_manager_node->get_current_state();

        if (std::chrono::steady_clock::now() - start_time >= timeout_duration) {
          RCLCPP_WARN(rclcpp::get_logger("deactivate_and_cleanup_node"),
                      "Timeout waiting for deactivation");
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }

    // After deactivation, check the state
    current_state = clips_env_manager_node->get_current_state();
    // Now trigger cleanup transition if the state is INACTIVE
    if (current_state.id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      clips_env_manager_node->trigger_transition(
          lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);

      auto start_time = std::chrono::steady_clock::now();

      while (current_state.id() !=
             lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
        current_state = clips_env_manager_node->get_current_state();

        if (std::chrono::steady_clock::now() - start_time >= timeout_duration) {
          RCLCPP_WARN(rclcpp::get_logger("deactivate_and_cleanup_node"),
                      "Timeout waiting for cleanup");
          break;
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(10)); // Avoid busy waiting
      }
    }
  }
  if (exe && clips_env_manager_node) {
    exe->remove_node(clips_env_manager_node->get_node_base_interface());
  }
  if (exe) {
    exe->cancel();
  }
  exe.reset();
  rclcpp::shutdown();
  clips_env_manager_node.reset();
}

int main(int argc, const char **argv) {

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  exe = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  std::signal(SIGINT, custom_signal_handler);

  clips_env_manager_node = std::make_shared<cx::CLIPSEnvManager>();

  exe->add_node(clips_env_manager_node->get_node_base_interface());

  clips_env_manager_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  clips_env_manager_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  exe->spin();
  custom_signal_handler(SIGINT);
  return 0;
}
