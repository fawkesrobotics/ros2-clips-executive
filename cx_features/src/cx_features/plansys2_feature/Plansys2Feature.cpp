#include <string>

#include "cx_features/Plansys2Feature.hpp"

#include "plansys2_problem_expert/ProblemExpertClient.hpp"


// To export as plugin
// #include "pluginlib/class_list_macros.hpp"

using std::placeholders::_1;
namespace cx {
Plansys2Feature::Plansys2Feature() {}

Plansys2Feature::~Plansys2Feature() {}

std::string Plansys2Feature::getFeatureName() const {
  return clips_feature_name;
}

void Plansys2Feature::initialise(const std::string &feature_name) {
  clips_feature_name = feature_name;
}

bool Plansys2Feature::clips_context_init(
    const std::string &env_name, LockSharedPtr<CLIPS::Environment> &clips) {
  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Initialising context for feature %s",
              clips_feature_name.c_str());
  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Initialising context for feature %s",
              clips_feature_name.c_str());
  envs_[env_name] = clips;

  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name), "Initialised context!");
  return true;
}

bool Plansys2Feature::clips_context_destroyed(
    const std::string &env_name, LockSharedPtr<CLIPS::Environment> &clips) {

  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Destroying clips context!");
  envs_.erase(env_name);

  return true;
}
} // namespace cx

// PLUGINLIB_EXPORT_CLASS(cx::Plansys2Feature, cx::ClipsFeature)
