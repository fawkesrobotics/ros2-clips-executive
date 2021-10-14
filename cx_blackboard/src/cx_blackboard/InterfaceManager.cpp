/***************************************************************************
 *  InterfaceManager.cpp
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

#include "cx_blackboard/InterfaceManager.hpp"

#include <algorithm>
#include <map>
#include <string>
#include <variant>

#include "rclcpp/rclcpp.hpp"

#include "cx_blackboard/Interface.hpp"
#include "cx_blackboard/InterfaceNode.hpp"

namespace cx {

InterfaceManager::InterfaceManager() {

  RCLCPP_INFO(rclcpp::get_logger("Interface_Manager"),
              "Initialsing Interface Manager...");

  RCLCPP_INFO(rclcpp::get_logger("Interface_Manager"),
              "Initialsied Interface Manager!");
}

// InterfaceManager::~InterfaceManager() {}

void InterfaceManager::generate_interface_node(const std::string &i_name,
                                               const std::string &i_type,
                                               const std::string &i_id) {
  RCLCPP_INFO(rclcpp::get_logger("Interface_Manager"),
              "Trying to generate interface of name: %s, type: %s, id %s...",
              i_name.c_str(), i_type.c_str(), i_id.c_str());

  auto it = interface_map_.find(i_type);
  if (it == interface_map_.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("Interface_Manager"),
                 "No such interface has been registered in the interface "
                 "manager map! name: %s, type: %s, id %s...",
                 i_name.c_str(), i_type.c_str(), i_id.c_str());
    throw std::runtime_error("No such interface registered!");
  }

  int interface_index = it->second;
  InterfaceManager::Interface_Node_Types_Variant generated_interface_node;

  switch (interface_index) {

  case RegisteredInterfaces::INTERFACE_PddlGenInterface: {

    std::shared_ptr<cx::Interface<cx_msgs::msg::PddlGenInterface>>
        generated_interface =
            std::make_shared<cx::Interface<cx_msgs::msg::PddlGenInterface>>(
                i_type, i_id);

    generated_interface_node =
        std::make_shared<cx::InterfaceNode<cx_msgs::srv::PddlGenInterfaceSrv,
                                           cx_msgs::msg::PddlGenInterface>>(
            i_type, generated_interface);

    interfaces_[i_type] = generated_interface_node;
  }
  case RegisteredInterfaces::INTERFACE_PddlPlannerInterface: {
    break;
  }
  default:
    break;
  }
  if (interfaces_.find(i_type) != interfaces_.end()) {

    threads_list_.push_front(std::make_unique<cx::NodeThread>(
        cast_interface(i_type).value()->get_node_base_interface()));
  }

  RCLCPP_INFO(rclcpp::get_logger("Interface_Manager"),
              "Generated interface of name: %s, type: %s, id %s...",
              i_name.c_str(), i_type.c_str(), i_id.c_str());
}

InterfaceManager::Interface_Node_Types_Optional
InterfaceManager::cast_interface(const std::string &i_type) {
  auto index = interfaces_.find(i_type);
  if (index == interfaces_.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("Interface_Manager"),
                 "No such interface of the given type: %s", i_type.c_str());
    return nullptr;
  }

  auto interface_index = interface_map_.find(i_type)->second;
  Interface_Node_Types_Optional ret_iface;

  try {
    switch (interface_index) {

    case RegisteredInterfaces::INTERFACE_PddlGenInterface: {
      ret_iface = std::get<std::shared_ptr<cx::InterfaceNode<
          cx_msgs::srv::PddlGenInterfaceSrv, cx_msgs::msg::PddlGenInterface>>>(
          interfaces_[i_type]);
      break;
    }

    case RegisteredInterfaces::INTERFACE_PddlPlannerInterface: {
      break;
    }
    }
  } catch (const std::bad_variant_access &e) {
    RCLCPP_ERROR(rclcpp::get_logger("Interface_Manager"),
                 "Bad interface cast for the given type: %s", i_type.c_str());
    std::cerr << e.what() << '\n';
    return nullptr;
  }

  return ret_iface;
}

void InterfaceManager::set_interface_writer(const std::string &i_type,
                                            const std::string &i_writer) {
  auto iface = cast_interface(i_type).value();
  if (iface == nullptr) {
    return;
  }

  if (iface->interface_->has_writer()) {
    RCLCPP_ERROR(rclcpp::get_logger("Interface_Manager"),
                 "There is already existing writer '%s' for interface %s! "
                 "Exception follows...",
                 i_writer.c_str(), i_type.c_str());
    throw std::runtime_error(
        "Error trying to overwrite existing interface writer for " + i_type +
        "!");
  }
  iface->interface_->set_writer(i_writer);
}

void InterfaceManager::set_interface_reader(const std::string &i_type,
                                            const std::string &i_reader) {
  auto iface = cast_interface(i_type).value();
  if (iface == nullptr) {
    return;
  }

  std::list<std::string> readers = std::move(iface->interface_->list_readers());
  if (std::find(readers.begin(), readers.end(), i_reader) != readers.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("Interface_Manager"),
                 "There is already existing writer '%s' for interface %s! "
                 "Exception follows...",
                 i_reader.c_str(), i_type.c_str());
    throw std::runtime_error(
        "Error trying to overwrite existing interface writer for " + i_type +
        "!");
  }
  iface->interface_->set_reader(i_reader);
}

} // namespace cx
