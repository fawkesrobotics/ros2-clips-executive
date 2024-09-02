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

#include "cx_clips/CLIPSEnvManagerNode.h"
#include "cx_clips_executive/ClipsExecutive.hpp"
#include "cx_feature_manager/ClipsFeaturesManager.hpp"

std::shared_ptr<cx::CLIPSEnvManagerNode> clips_env_manager_node;
std::shared_ptr<cx::ClipsFeaturesManager> clips_features_manager_node;
std::shared_ptr<cx::ClipsExecutive> clips_executive_node;

void custom_signal_handler(int /*signum*/) {
  // Ensure all nodes are properly deactivated and cleaned up
  if (clips_env_manager_node) {
    if (clips_env_manager_node->get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      clips_env_manager_node->trigger_transition(
          lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    }
    if (clips_env_manager_node->get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      clips_env_manager_node->trigger_transition(
          lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
    }
  }

  if (clips_features_manager_node) {
    if (clips_features_manager_node->get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      clips_features_manager_node->trigger_transition(
          lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    }
    if (clips_features_manager_node->get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      clips_features_manager_node->trigger_transition(
          lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
    }
  }

  if (clips_executive_node) {
    if (clips_executive_node->get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      clips_executive_node->trigger_transition(
          lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    }
    if (clips_executive_node->get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      clips_executive_node->trigger_transition(
          lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
    }
  }

  // Wait for a short period to ensure transitions complete
  std::this_thread::sleep_for(std::chrono::seconds(1));
  rclcpp::shutdown();
}

int main(int argc, const char **argv) {

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  clips_env_manager_node = std::make_shared<cx::CLIPSEnvManagerNode>();
  clips_features_manager_node = std::make_shared<cx::ClipsFeaturesManager>();
  signal(SIGINT, custom_signal_handler);
  clips_executive_node = std::make_shared<cx::ClipsExecutive>();

  exe.add_node(clips_env_manager_node->get_node_base_interface());
  exe.add_node(clips_features_manager_node->get_node_base_interface());
  exe.add_node(clips_executive_node->get_node_base_interface());

  clips_features_manager_node->pre_configure(clips_env_manager_node);
  clips_executive_node->pre_configure(clips_env_manager_node);

  clips_env_manager_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  clips_env_manager_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  exe.spin();
  custom_signal_handler(SIGINT);
  return 0;
}
