/***************************************************************************
 *  BlackboardNode.hpp
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

#ifndef CX_BLACKBOARD__BLACKBOARDNODE_HPP_
#define CX_BLACKBOARD__BLACKBOARDNODE_HPP_

#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "cx_blackboard/InterfaceManager.hpp"

#include "cx_msgs/srv/open_interface.hpp"

namespace cx {
class Blackboard : public rclcpp_lifecycle::LifecycleNode {
public:
  Blackboard();
  // ~Blackboard();

  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state);
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
  // CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);
  // CallbackReturn on_error(const rclcpp_lifecycle::State &state);

  void open_interface_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<cx_msgs::srv::OpenInterface::Request> request,
      const std::shared_ptr<cx_msgs::srv::OpenInterface::Response> response);

  std::unique_ptr<cx::InterfaceManager> imngr_;

private:
  rclcpp::Service<cx_msgs::srv::OpenInterface>::SharedPtr
      open_interface_service_;
};
} // namespace cx

#endif // !CX_BLACKBOARD__BLACKBOARDNODE_HPP_