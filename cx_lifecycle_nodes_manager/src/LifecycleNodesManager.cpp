/***************************************************************************
 *  LifecycleNodesManager.cpp
 *
 *  Created: 28 June 2021
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

#include "cx_lifecycle_nodes_manager/LifecycleNodesManager.hpp"

#include "rclcpp/rclcpp.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

using namespace std::chrono_literals;

namespace cx {

LifecycleNodesManager::LifecycleNodesManager()
    : Node("lifecycle_nodes_manager") {
  RCLCPP_INFO(get_logger(), "Initialising...");

  declare_parameter("node_names_to_manage", node_names_to_manage_);

  // GEET THE SPECIFIED NODES FROM THE LAUNCH CONFIGURATION
  node_names_to_manage_ =
      get_parameter("node_names_to_manage").as_string_array();
}

LifecycleNodesManager::~LifecycleNodesManager() {}

void LifecycleNodesManager::initialise() {
  RCLCPP_INFO(get_logger(), "Creating lifecycle nodes clients...");

  for (auto &lc_node_name : node_names_to_manage_) {
    nodes_to_manage_[lc_node_name] = std::make_shared<cx::LifecycleNodesClient>(
        shared_from_this(), lc_node_name);
  }

  startupScript();
}

bool LifecycleNodesManager::startupScript() {
  RCLCPP_INFO(get_logger(), "Starting all specified nodes...");

  if (!changeAllNodeStates(
          lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE) ||
      !changeAllNodeStates(
          lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
    RCLCPP_ERROR(get_logger(),
                 "Couldn't configure/activate all nodes! aborting...");
    return false;
  }

  RCLCPP_INFO(get_logger(), "Started all specified nodes!");
  return true;
}

bool LifecycleNodesManager::changeNodeState(const std::string &lc_node_name,
                                            std::uint8_t transition) {
  RCLCPP_DEBUG(get_logger(), "Changing state of %s", lc_node_name.c_str());

  if (!nodes_to_manage_[lc_node_name]->change_node_state(transition)
      // ||
      //     !(nodes_to_manage_[lc_node_name]->get_node_state() ==
      //       static_cast<uint8_t>(transition))
  ) {
    RCLCPP_ERROR(get_logger(), "Changing the state for %s failed",
                 lc_node_name.c_str());
    return false;
  }
  // TEST PURPOSE
  nodes_to_manage_[lc_node_name]->get_node_state();
  RCLCPP_DEBUG(get_logger(), "Changed state of %s ", lc_node_name.c_str());
  return true;
}

bool LifecycleNodesManager::changeAllNodeStates(std::uint8_t transition) {

  if (transition == lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE ||
      transition == lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE) {
    for (auto &lc_node_name : node_names_to_manage_) {
      if (!changeNodeState(lc_node_name, transition)) {
        return false;
      }
    }
  } else {
    // MAYBE CHANGE STATES IN REVERSE ORDERS TO AVOID DEPENDENCIES BREAKING!
    for (auto &lc_node_name : node_names_to_manage_) {
      if (!changeNodeState(lc_node_name, transition)) {
        return false;
      }
    }
  }
  return true;
}

void LifecycleNodesManager::disableAllNodes() {

  RCLCPP_INFO(get_logger(), "Shutting down all lc nodes!");

  changeAllNodeStates(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  changeAllNodeStates(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
  changeAllNodeStates(
      lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
}

} // namespace cx
