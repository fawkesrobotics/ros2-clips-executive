#include <map>
#include <memory>
#include <string>
#include <utility>

#include "cx_features/ClipsFeaturesManager.hpp"
#include "cx_features/MockFeature.hpp"

#include "cx_core/ClipsFeature.hpp"
#include "cx_msgs/srv/clips_feature_context.hpp"

#include "lifecycle_msgs/msg/state.hpp"

using namespace std::placeholders;

namespace cx {

ClipsFeaturesManager::ClipsFeaturesManager()
    : rclcpp_lifecycle::LifecycleNode("clips_features_manager") {}

ClipsFeaturesManager::~ClipsFeaturesManager() {}

void ClipsFeaturesManager::pre_configure(
    std::shared_ptr<cx::CLIPSEnvManagerNode> &manager_node) {
  clips_env_manager_node_ = manager_node;
}

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn
ClipsFeaturesManager::on_configure(const rclcpp_lifecycle::State &state) {

  RCLCPP_INFO(get_logger(), "Configuring [%s]...", get_name());

  feature_init_context_service_ =
      create_service<cx_msgs::srv::ClipsFeatureContext>(
          "clips_features_manager/init_feature_context",
          std::bind(&ClipsFeaturesManager::feature_init_context_callback, this,
                    _1, _2, _3));

  feature_destroy_context_service_ =
      create_service<cx_msgs::srv::ClipsFeatureContext>(
          "clips_features_manager/destroy_feature_context",
          std::bind(&ClipsFeaturesManager::feature_destroy_context_callback,
                    this, _1, _2, _3));

  // GET THE REQUESTED FEATURES FROM THE PARAMETERS LATER!!!!!!
  auto mock_feature = std::make_shared<cx::MockFeature>("mock_feature");
  features_[mock_feature->getFeatureName()] = std::move(mock_feature);
  auto mock_feature_1 = std::make_shared<cx::MockFeature>("mock_feature_1");
  features_[mock_feature_1->getFeatureName()] = std::move(mock_feature_1);
  auto mock_feature_2 = std::make_shared<cx::MockFeature>("mock_feature_2");
  features_[mock_feature_2->getFeatureName()] = std::move(mock_feature_2);
  auto mock_feature_3 = std::make_shared<cx::MockFeature>("mock_feature_3");
  features_[mock_feature_3->getFeatureName()] = std::move(mock_feature_3);
  auto mock_feature_4 = std::make_shared<cx::MockFeature>("mock_feature_4");
  features_[mock_feature_4->getFeatureName()] = std::move(mock_feature_4);

  // auto mock_feature_5 = std::make_shared<cx::MockFeature>("mock_feature_5");
  // auto mock_feature_6 = std::make_shared<cx::MockFeature>("mock_feature_6");
  // auto mock_feature_7 = std::make_shared<cx::MockFeature>("mock_feature_7");
  // auto mock_feature_8 = std::make_shared<cx::MockFeature>("mock_feature_8");

  RCLCPP_INFO(get_logger(), "Configured [%s]!", get_name());
  return CallbackReturn::SUCCESS;
}

CallbackReturn
ClipsFeaturesManager::on_activate(const rclcpp_lifecycle::State &state) {

  RCLCPP_INFO(get_logger(), "Activating [%s]...", get_name());
  RCLCPP_INFO(get_logger(), "Activated [%s]!", get_name());
  return CallbackReturn::SUCCESS;
}

void ClipsFeaturesManager::feature_init_context_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<cx_msgs::srv::ClipsFeatureContext::Request> request,
    const std::shared_ptr<cx_msgs::srv::ClipsFeatureContext::Response>
        response) {

  const std::string &env_name = request->env_name;
  const std::string &feature_name = request->feature_name;

  LockSharedPtr<CLIPS::Environment> &clips =
      clips_env_manager_node_->envs_[env_name].env;

  if (features_.find(feature_name) != features_.end()) {
    bool success = features_[feature_name]->clips_context_init(env_name, clips);
    if (!success) {
      response->error =
          "Error by context initialisation: feature " + feature_name;
    }

    response->success = success;
    return;

  } else {
    RCLCPP_ERROR(get_logger(), "%s is not registered!", feature_name.c_str());
    response->error = feature_name + " has not been registered!";
    response->success = false;
  }
}

void ClipsFeaturesManager::feature_destroy_context_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<cx_msgs::srv::ClipsFeatureContext::Request> request,
    const std::shared_ptr<cx_msgs::srv::ClipsFeatureContext::Response>
        response) {

  const std::string &env_name = request->env_name;
  const std::string &feature_name = request->feature_name;
  RCLCPP_INFO(get_logger(), "IN DESTROY CONTEXT CALLBACK FOR FEATURE %s",
              feature_name.c_str());

  LockSharedPtr<CLIPS::Environment> &clips =
      clips_env_manager_node_->envs_[env_name].env;

  if (features_.find(feature_name) != features_.end()) {
    RCLCPP_INFO(get_logger(), "IN IF FOR CONTEXT DESTROY");
    bool success =
        features_[feature_name]->clips_context_destroyed(env_name, clips);
    if (!success) {
      response->error = "Error by context destruction: feature " + feature_name;
    }
    response->success = success;
    return;
  } else {
    RCLCPP_ERROR(get_logger(), "%s is not registered!", feature_name.c_str());
    response->error = feature_name + " has not been registered!";
  }
  response->success = false;
}
} // namespace cx
