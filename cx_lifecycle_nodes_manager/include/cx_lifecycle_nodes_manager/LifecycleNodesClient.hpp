/***************************************************************************
 *  LifecycleNodesClient.hpp
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

#ifndef CX_LIFECYCLE_NODES_MANAGER__LIFECYCLENODESCLIENT_HPP_
#define CX_LIFECYCLE_NODES_MANAGER__LIFECYCLENODESCLIENT_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

using std::chrono::seconds;

namespace cx {

class LifecycleNodesClient {
public:
  // Initialises a node with the given name and client service calls execution
  explicit LifecycleNodesClient(const std::string &node_name,
                                const std::string &managed_node_name);
  // Utilises the given node for logging and client service calls execution
  explicit LifecycleNodesClient(std::shared_ptr<rclcpp::Node> controller_node,
                                const std::string &managed_node_name);
  ~LifecycleNodesClient();
  uint8_t get_node_state(seconds timeout = seconds(5));
  bool change_node_state(std::uint8_t transition, seconds timeout = seconds(10));

private:
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>>
      get_node_state_client_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>>
      change_node_state_client_;

  rclcpp::Node::SharedPtr node_;

  const std::string node_name_;
  const std::string managed_node_name_;

  // Callback group so it run the clients requests synch
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
};

} // namespace cx

#endif // !CX_LIFECYCLE_NODES_MANAGER__LIFECYCLENODESCLIENT_HPP_
