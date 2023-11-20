/***************************************************************************
 *  LifecycleNodesClient.cpp
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

#include <memory>
#include <string>

#include "cx_lifecycle_nodes_manager/LifecycleNodesClient.hpp"

#include "rclcpp/rclcpp.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

using std::chrono::seconds;
using namespace std::chrono_literals;

namespace cx {

LifecycleNodesClient::LifecycleNodesClient(const std::string &node_name,
                                           const std::string &managed_node_name)
    : node_name_{node_name}, managed_node_name_{managed_node_name} {
  node_ = rclcpp::Node::make_shared(node_name_);

  callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_executor_.add_callback_group(callback_group_,
                                              node_->get_node_base_interface());

  RCLCPP_INFO(node_->get_logger(), "Creating client for service %s/get_state",
              managed_node_name.c_str());
  get_node_state_client_ = node_->create_client<lifecycle_msgs::srv::GetState>(
      managed_node_name + "/get_state", rmw_qos_profile_services_default,
      callback_group_);

  RCLCPP_INFO(node_->get_logger(),
              "Creating client for service %s/change_state",
              managed_node_name.c_str());
  change_node_state_client_ =
      node_->create_client<lifecycle_msgs::srv::ChangeState>(
          managed_node_name + "/change_state", rmw_qos_profile_services_default,
          callback_group_);
}

LifecycleNodesClient::LifecycleNodesClient(
    std::shared_ptr<rclcpp::Node> controller_node,
    const std::string &managed_node_name)
    : managed_node_name_{managed_node_name} {
  node_ = controller_node;

  callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_executor_.add_callback_group(callback_group_,
                                              node_->get_node_base_interface());

  RCLCPP_INFO(node_->get_logger(), "Creating client for service %s/get_state",
              managed_node_name.c_str());
  get_node_state_client_ = node_->create_client<lifecycle_msgs::srv::GetState>(
      managed_node_name + "/get_state", rmw_qos_profile_services_default,
      callback_group_);

  RCLCPP_INFO(node_->get_logger(),
              "Creating client for service %s/change_state",
              managed_node_name.c_str());
  change_node_state_client_ =
      node_->create_client<lifecycle_msgs::srv::ChangeState>(
          managed_node_name + "/change_state", rmw_qos_profile_services_default,
          callback_group_);
}

LifecycleNodesClient::~LifecycleNodesClient() {
  RCLCPP_INFO(node_->get_logger(), "Destroying...");
}

uint8_t LifecycleNodesClient::get_node_state(seconds timeout) {
  RCLCPP_INFO(node_->get_logger(), "In Get Node State");
  while (!get_node_state_client_->wait_for_service(2s)) {
    if (!rclcpp::ok()) {

      RCLCPP_ERROR(node_->get_logger(),
                   "%s: timed out waiting for service availability",
                   get_node_state_client_->get_service_name());
      return false;
    }
    RCLCPP_WARN(node_->get_logger(), "%s: still waiting for service...",
                get_node_state_client_->get_service_name());
  }
  RCLCPP_INFO(node_->get_logger(), "%s service client: sending async request",
              get_node_state_client_->get_service_name());

  auto req = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto future_res = get_node_state_client_->async_send_request(req);

  if (callback_group_executor_.spin_until_future_complete(
          future_res, timeout) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Failed to get current state for node %s ",
                 managed_node_name_.c_str());
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }

  auto res = future_res.get();

  RCLCPP_INFO(node_->get_logger(), "Node %s current state: %s.",
              managed_node_name_.c_str(), res->current_state.label.c_str());
  return res->current_state.id;
}

bool LifecycleNodesClient::change_node_state(std::uint8_t transition,
                                             seconds timeout) {
  RCLCPP_INFO(node_->get_logger(), "In Change Node State");

  while (!change_node_state_client_->wait_for_service(2s)) {
    if (!rclcpp::ok()) {

      RCLCPP_ERROR(node_->get_logger(),
                   "%s: timed out waiting for service availability",
                   change_node_state_client_->get_service_name());
      return false;
    }
    RCLCPP_WARN(node_->get_logger(), "%s: still waiting for service...",
                change_node_state_client_->get_service_name());
  }

  RCLCPP_INFO(node_->get_logger(), "%s service client: sending async request",
              change_node_state_client_->get_service_name());

  auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  req->transition.id = transition;

  auto future_res = change_node_state_client_->async_send_request(req);

  if (callback_group_executor_.spin_until_future_complete(
          future_res, timeout) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "Time out waiting for service %s -> Failed to trigger transition %u",
        change_node_state_client_->get_service_name(),
        static_cast<uint8_t>(transition));
    return false;
  }
  if (future_res.get()->success) {
    RCLCPP_INFO(node_->get_logger(), "%s Succesful transition to %d",
                managed_node_name_.c_str(), static_cast<int>(transition));
    return true;
  } else {
    RCLCPP_WARN(node_->get_logger(), "Failed to trigger transition %u",
                static_cast<uint8_t>(transition));
    return false;
  }
}
} // namespace cx
