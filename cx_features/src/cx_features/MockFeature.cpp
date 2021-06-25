#include <string>

#include "cx_features/MockFeature.hpp"

using std::placeholders::_1;
namespace cx {
MockFeature::MockFeature(const std::string &feature_name)
    : ClipsFeature(feature_name) {

  // init_context_sub_ = create_subscription<cx_msgs::msg::ClipsContext>(
  //     "clips_manager/request_feature_context_initialisation",
  //     rclcpp::QoS(100).keep_all(),
  //     std::bind(&MockFeature::clips_context_init, this, _1));
  // destroy_context_sub_ = create_publisher<cx_msgs::msg::ClipsContext>(
  //     "clips_manager/request_feature_context_destroy",
  //     rclcpp::QoS(20).reliable(),
  //     std::bind(&MockFeature::clips_context_destroyed, this, _1));
}

MockFeature::~MockFeature() {}

std::string MockFeature::getFeatureName() const { return clips_feature_name; }

// void MockFeature::addEnvironment(const std::string &env_name,
//                                  LockSharedPtr<CLIPS::Environment> &clips) {
//   RCLCPP_WARN(get_logger(), "Adding env %s to map of %s", env_name.c_str(),
//               clips_feature_name.c_str());
//   envs_[env_name] = clips;
// }

bool MockFeature::clips_context_init(const std::string &env_name,
                                     LockSharedPtr<CLIPS::Environment> &clips) {
  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Initialising context for feature %s",
              clips_feature_name.c_str());

  clips.scopedLock();
  clips->unwatch("all");
  clips->watch("all");
  clips->reset();
  clips->assert_fact("(b (name " + clips_feature_name + "))");
  clips->run(-1);

  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Initialised the context!");
  return true;
}
bool MockFeature::clips_context_destroyed(
    const std::string &env_name, LockSharedPtr<CLIPS::Environment> &clips) {

  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Destroying clips context!");
  return true;
}
} // namespace cx
