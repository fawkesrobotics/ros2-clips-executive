// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#include <string>

#include "cx_example_plugin/example_plugin.hpp"
#include <cx_utils/clips_env_context.hpp>

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

using std::placeholders::_1;
namespace cx {
ExamplePlugin::ExamplePlugin() {}

ExamplePlugin::~ExamplePlugin() {}

void ExamplePlugin::initialize() {
  logger_ = std::make_unique<rclcpp::Logger>(rclcpp::get_logger(plugin_name_));
}

bool ExamplePlugin::clips_env_init(LockSharedPtr<clips::Environment> &env) {
  auto context = CLIPSEnvContext::get_context(env.get_obj().get());
  RCLCPP_INFO(*logger_, "Initializing plugin for environment %s",
              context->env_name_.c_str());
  return true;
}

bool ExamplePlugin::clips_env_destroyed(
    LockSharedPtr<clips::Environment> &env) {
  auto context = CLIPSEnvContext::get_context(env.get_obj().get());
  RCLCPP_INFO(*logger_, "Destroying plugin for environment %s",
              context->env_name_.c_str());
  return true;
}
} // namespace cx

PLUGINLIB_EXPORT_CLASS(cx::ExamplePlugin, cx::ClipsPlugin)
