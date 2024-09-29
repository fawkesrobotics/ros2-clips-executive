// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#include <string>

#include "cx_example_feature/example_feature.hpp"

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

using std::placeholders::_1;
namespace cx {
ExampleFeature::ExampleFeature() {}

ExampleFeature::~ExampleFeature() {}

bool ExampleFeature::clips_context_init(
    const std::string &env_name, LockSharedPtr<clips::Environment> &clips) {
  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name_),
              "Initialising context for feature %s",
              clips_feature_name_.c_str());

  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name_),
              "Initialised the context for env %s!", env_name.c_str());
  (void)clips;
  return true;
}

bool ExampleFeature::clips_context_destroyed(const std::string &env_name) {

  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name_),
              "Destroying clips context for env %s!", env_name.c_str());
  return true;
}
} // namespace cx

PLUGINLIB_EXPORT_CLASS(cx::ExampleFeature, cx::ClipsFeature)
