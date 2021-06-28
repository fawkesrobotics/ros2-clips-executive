#include <map>
#include <memory>
#include <string>
#include <utility>

#include "cx_features/ClipsFeaturesManager.hpp"
#include "cx_features/MockFeature.hpp"

#include "cx_core/ClipsFeature.hpp"
#include "cx_msgs/srv/clips_feature_context.hpp"
#include "cx_utils/params_utils.hpp"

#include "lifecycle_msgs/msg/state.hpp"

using namespace std::placeholders;

namespace cx {

ClipsFeaturesManager::ClipsFeaturesManager()
    : rclcpp_lifecycle::LifecycleNode("clips_features_manager"),
      env_manager_client_{std::make_shared<cx::CLIPSEnvManagerClient>()},
      pg_loader_("cx_core", "cx::ClipsFeature"), default_ids_{},
      default_types_{} {

  declare_parameter("clips_features", default_ids_);
}

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

  auto node = shared_from_this();

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

  // Load all registered features in the cx_params.yml file
  get_parameter("clips_features", features_ids_);

  if (!features_ids_.empty()) {
    features_types_.resize(features_ids_.size());

    for (size_t i = 0; i < features_types_.size(); i++) {
      try {
        const std::string feat_name = features_ids_[i];
        features_types_[i] = cx::get_plugin_type_param(node, feat_name);

        cx::ClipsFeature::Ptr feature =
            pg_loader_.createUniqueInstance(features_types_[i]);

        // Call the feature initialisation
        feature->initialise(feat_name);

        RCLCPP_INFO(get_logger(), "Created feature : %s of type %s",
                    feat_name.c_str(), features_types_[i].c_str());
        // Insert loaded feature to the features map
        features_.insert({feat_name, feature});

      } catch (const pluginlib::PluginlibException &ex) {
        RCLCPP_FATAL(get_logger(), "Failed to load feature. Exception: %s",
                     ex.what());
        exit(-1);
      }
    }
  } else {
    RCLCPP_WARN(
        get_logger(),
        "Loading Clips Feature Manager without any features specified!");
  }
  // populate the vector of feature names
  for (auto &feat : features_) {
    feature_names_vector_.emplace_back(feat.second->getFeatureName());
  }
  RCLCPP_INFO(get_logger(), "Sending features to Clips Environment Manager");
  // Send all available features to the env manager!
  env_manager_client_->addFeatures(std::move(feature_names_vector_));
  
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
