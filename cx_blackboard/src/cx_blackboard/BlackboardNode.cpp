/***************************************************************************
 *  BlackboardNode.cpp
 *
 *  Created: 28 August 2021
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

#include "cx_blackboard/BlackboardNode.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

namespace cx {

Blackboard::Blackboard() : rclcpp_lifecycle::LifecycleNode("blackboard") {
  RCLCPP_INFO(get_logger(), "Initialising [%s]...", get_name());

  RCLCPP_INFO(get_logger(), "Initialisied[%s]!", get_name());
}

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn Blackboard::on_configure(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Configuring [%s]...", get_name());

  imngr_ = std::make_unique<cx::InterfaceManager>();

  open_interface_service_ = create_service<cx_msgs::srv::OpenInterface>(
      "blackboard/open_interface",
      std::bind(&Blackboard::open_interface_callback, this, _1, _2, _3));

  RCLCPP_INFO(get_logger(), "Configured [%s]!", get_name());
  return CallbackReturn::SUCCESS;
}
CallbackReturn Blackboard::on_activate(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Activating [%s]...", get_name());
  RCLCPP_INFO(get_logger(), "Activated [%s]!", get_name());
  return CallbackReturn::SUCCESS;
}
CallbackReturn Blackboard::on_deactivate(const rclcpp_lifecycle::State &state) {

  RCLCPP_INFO(get_logger(), "Deactivating [%s]...  ", get_name());
  RCLCPP_INFO(get_logger(), "Deactivated [%s]!", get_name());
  return CallbackReturn::SUCCESS;
}

void Blackboard::open_interface_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<cx_msgs::srv::OpenInterface::Request> request,
    const std::shared_ptr<cx_msgs::srv::OpenInterface::Response> response) {

  RCLCPP_INFO(get_logger(), "Entering");
  std::string &i_type = request->i_type;
  std::string &i_id = request->i_id;
  std::string &i_owner = request->i_owner;
  bool writing = request->writing;

  // No such Interface registered yet
  if (imngr_->interfaces_.find(i_type) == imngr_->interfaces_.end()) {
    try {
      imngr_->generate_interface_node(i_type, i_type, i_id);
    } catch (const std::runtime_error &e) {
      RCLCPP_ERROR_STREAM(get_logger(), e.what());
      response->success = false;
      return;
    }
  }

  if (writing) {
    imngr_->set_interface_writer(i_type, i_owner);
  } else {
    imngr_->set_interface_reader(i_type, i_owner);
  }
  response->success = true;
}
} // namespace cx