// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#include <string>

#include "cx_example_feature/example_feature.hpp"
#include <cx_utils/clips_env_context.hpp>

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

using std::placeholders::_1;
namespace cx {
ExampleFeature::ExampleFeature() {}

ExampleFeature::~ExampleFeature() {}

void ExampleFeature::initialize() {
  logger_ = std::make_unique<rclcpp::Logger>(rclcpp::get_logger(feature_name_));
}

bool ExampleFeature::clips_env_init(LockSharedPtr<clips::Environment> &env) {
  auto context = CLIPSEnvContext::get_context(env.get_obj().get());
  RCLCPP_INFO(*logger_, "Initializing feature for environment %s",
              context->env_name_.c_str());
  return true;
}

bool ExampleFeature::clips_env_destroyed(
    LockSharedPtr<clips::Environment> &env) {
  auto context = CLIPSEnvContext::get_context(env.get_obj().get());
  RCLCPP_INFO(*logger_, "Destroying feature for environment %s",
              context->env_name_.c_str());
  return true;
}
} // namespace cx

PLUGINLIB_EXPORT_CLASS(cx::ExampleFeature, cx::ClipsFeature)
