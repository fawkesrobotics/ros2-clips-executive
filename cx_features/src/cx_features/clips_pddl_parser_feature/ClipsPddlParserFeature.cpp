#include <map>
#include <memory>
#include <string>

#include <clips_pddl_parser/clips_pddl_parser.h>

#include "cx_core/ClipsFeature.hpp"
#include "cx_features/ClipsPddlParserFeature.hpp"
#include "cx_utils/LockSharedPtr.hpp"

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

namespace cx {

ClipsPddlParserFeature::ClipsPddlParserFeature() {}
ClipsPddlParserFeature::~ClipsPddlParserFeature() {}

std::string ClipsPddlParserFeature::getFeatureName() const {
  return clips_feature_name;
}

void ClipsPddlParserFeature::initialise(const std::string &feature_name) {
  clips_feature_name = feature_name;
}

bool ClipsPddlParserFeature::clips_context_init(
    const std::string &env_name, LockSharedPtr<CLIPS::Environment> &clips) {
  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Initialising context for feature %s",
              clips_feature_name.c_str());

  envs_[env_name] = clips;
  RCLCPP_WARN(rclcpp::get_logger(clips_feature_name), "Trying to lock mutex");
  pddl_parser = std::make_unique<clips_pddl_parser::ClipsPddlParser>(
      envs_[env_name].get_obj().get(), *(envs_[env_name].get_mutex_instance()),
      false);

  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name), "Initialised context!");
  return true;
}

bool ClipsPddlParserFeature::clips_context_destroyed(
    const std::string &env_name, LockSharedPtr<CLIPS::Environment> &clips) {

  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Destroying clips context!");
  envs_.erase(env_name);
  delete maintained_env_;

  return true;
}
} // namespace cx
PLUGINLIB_EXPORT_CLASS(cx::ClipsPddlParserFeature, cx::ClipsFeature)
