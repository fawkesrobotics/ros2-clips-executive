#include <string>

#include "cx_features/MockFeature.hpp"

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

using std::placeholders::_1;
namespace cx {
MockFeature::MockFeature() {}

MockFeature::~MockFeature() {}

std::string MockFeature::getFeatureName() const { return clips_feature_name; }

void MockFeature::initialise(const std::string &feature_name) {
  clips_feature_name = feature_name;
}

bool MockFeature::clips_context_init(const std::string &env_name,
                                     LockSharedPtr<CLIPS::Environment> &clips) {
  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Initialising context for feature %s",
              clips_feature_name.c_str());

  std::lock_guard<std::mutex> guard(*(clips.get_mutex_instance()));
  clips->unwatch("all");
  clips->watch("all");
  clips->reset();
  clips->assert_fact("(b (name " + clips_feature_name + "))");
  clips->run(-1);

  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Initialised the context for env %s!", env_name.c_str());
  return true;
}

bool MockFeature::clips_context_destroyed(const std::string &env_name) {

  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Destroying clips context for env %s!", env_name.c_str());
  return true;
}
} // namespace cx

PLUGINLIB_EXPORT_CLASS(cx::MockFeature, cx::ClipsFeature)
