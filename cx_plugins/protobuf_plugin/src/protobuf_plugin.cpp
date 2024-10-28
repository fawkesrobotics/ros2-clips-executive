// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  ProtobufPlugin.cpp
 *
 *  Created: October 13th, 2023
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

#include <map>
#include <memory>
#include <string>

#include <clips_protobuf/communicator.h>

#include "cx_plugin/clips_plugin.hpp"
#include "cx_protobuf_plugin/protobuf_plugin.hpp"
#include <cx_utils/LockSharedPtr.hpp>
#include <cx_utils/clips_env_context.hpp>
#include <cx_utils/param_utils.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

namespace cx {

ProtobufPlugin::ProtobufPlugin() {}
ProtobufPlugin::~ProtobufPlugin() {}

void ProtobufPlugin::initialize() {
  auto node = parent_.lock();
  if (node) {
    std::vector<std::string> package_share_dirs, input_proto_paths;
    cx::cx_utils::declare_parameter_if_not_declared(
        node, plugin_name_ + ".proto_paths",
        rclcpp::ParameterValue(std::vector<std::string>()));
    node->get_parameter(plugin_name_ + ".proto_paths", input_proto_paths);
    cx::cx_utils::declare_parameter_if_not_declared(
        node, plugin_name_ + ".package_share_dirs",
        rclcpp::ParameterValue(std::vector<std::string>()));
    node->get_parameter(plugin_name_ + ".package_share_dirs",
                        package_share_dirs);
    logger_ =
        std::make_unique<rclcpp::Logger>(rclcpp::get_logger(plugin_name_));
    try {
      cx_utils::resolve_files(input_proto_paths, package_share_dirs, paths_);
    } catch (std::exception &e) {
      RCLCPP_ERROR(*logger_, e.what());
    }
  }
}

bool ProtobufPlugin::clips_env_init(LockSharedPtr<clips::Environment> &env) {
  RCLCPP_DEBUG(*logger_, "Initializing context for plugin %s",
               plugin_name_.c_str());
  auto context = CLIPSEnvContext::get_context(env.get_obj().get());
  protobuf_communicator_[context->env_name_] =
      std::make_unique<protobuf_clips::ClipsProtobufCommunicator>(
          env.get_obj().get(), *(env.get_mutex_instance()), paths_);

  RCLCPP_INFO(rclcpp::get_logger(plugin_name_), "Initialised context!");

  return true;
}

bool ProtobufPlugin::clips_env_destroyed(
    LockSharedPtr<clips::Environment> &env) {

  RCLCPP_INFO(rclcpp::get_logger(plugin_name_), "Destroying clips context!");

  auto context = CLIPSEnvContext::get_context(env.get_obj().get());
  protobuf_communicator_.erase(context->env_name_);
  return true;
}
} // namespace cx
PLUGINLIB_EXPORT_CLASS(cx::ProtobufPlugin, cx::ClipsPlugin)
