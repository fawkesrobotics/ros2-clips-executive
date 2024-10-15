// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#include <map>
#include <memory>
#include <ranges>
#include <string>
#include <utility>

#include "cx_feature_manager/clips_feature_manager.hpp"

#include "cx_feature/clips_feature.hpp"
#include "cx_msgs/srv/clips_feature_context.hpp"
#include "cx_utils/param_utils.hpp"

#include "lifecycle_msgs/msg/state.hpp"

using namespace std::placeholders;

namespace cx {

ClipsFeatureManager::ClipsFeatureManager()
    : pg_loader_("cx_feature", "cx::ClipsFeature") {}

ClipsFeatureManager::~ClipsFeatureManager() {}

void ClipsFeatureManager::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    const std::string &name, LockSharedPtr<EnvsMap> &envs) {
  parent_ = parent;
  name_ = name;
  envs_ = envs;
  auto node = parent_.lock();
  if (node) {
    RCLCPP_INFO(logger_, "Configuring [%s]...", name_.c_str());

    load_plugin_service_ = node->create_service<cx_msgs::srv::LoadCLIPSPlugin>(
        std::format("{}/load_plugin", name_).c_str(),
        std::bind(&ClipsFeatureManager::load_plugin_cb, this, _1, _2, _3));
    unload_plugin_service_ =
        node->create_service<cx_msgs::srv::UnloadCLIPSPlugin>(
            std::format("{}/unload_plugin", name_).c_str(),
            std::bind(&ClipsFeatureManager::unload_plugin_cb, this, _1, _2,
                      _3));
    list_plugin_service_ = node->create_service<cx_msgs::srv::ListCLIPSPlugin>(
        std::format("{}/list_plugin", name_).c_str(),
        std::bind(&ClipsFeatureManager::list_plugin_cb, this, _1, _2, _3));

    // Load all registered features in the cx_params.yml file
    cx::cx_utils::declare_parameter_if_not_declared(
        node, "clips_plugins", rclcpp::ParameterValue(feature_ids_));
    node->get_parameter("clips_plugins", feature_ids_);
    RCLCPP_INFO(logger_, "Detect features:");
    for (const auto &feat : feature_ids_) {
      RCLCPP_INFO(logger_, "Detected [ %s ]", feat.c_str());
    }

    if (!feature_ids_.empty()) {
      for (size_t i = 0; i < feature_ids_.size(); i++) {
        try {
          const std::string feat_type =
              cx::cx_utils::get_plugin_type_param(node, feature_ids_[i]);

          cx::ClipsFeature::Ptr feature =
              pg_loader_.createUniqueInstance(feat_type);

          feature->initialize(parent, feature_ids_[i]);

          RCLCPP_INFO(logger_, "Created feature : %s of type %s",
                      feature_ids_[i].c_str(), feat_type.c_str());
          // Insert loaded feature to the features map
          features_.insert({feature_ids_[i], std::move(feature)});
        } catch (const pluginlib::PluginlibException &ex) {
          RCLCPP_FATAL(logger_, "Failed to load feature. Exception: %s",
                       ex.what());
          exit(-1);
        }
      }
    } else {
      RCLCPP_WARN(logger_,
                  "Loading Clips Feature Manager without any features.");
    }
  } else {
    RCLCPP_ERROR(logger_, "Invalid parent node reference!");
  }
}

void ClipsFeatureManager::activate() {
  std::scoped_lock env_lock(*envs_.get_mutex_instance());
  for (auto &env : *envs_.get_obj().get()) {
    activate_env(env.first, env.second);
  }
}

void ClipsFeatureManager::activate_env(const std::string &env_name,
                                       LockSharedPtr<clips::Environment> &env) {

  RCLCPP_INFO(logger_, "Loading features for %s.", env_name.c_str());
  std::vector<std::string> plugins;
  auto node = parent_.lock();
  cx::cx_utils::declare_parameter_if_not_declared(
      node, env_name + ".plugins", rclcpp::ParameterValue(plugins));
  node->get_parameter(env_name + ".plugins", plugins);
  for (const auto &plugin : plugins) {
    bool success = false;
    if (features_.contains(plugin)) {
      success = features_[plugin]->clips_env_init(env);
    } else {
      std::string plugin_type;
      cx::cx_utils::declare_parameter_if_not_declared(
          node, plugin + ".plugin", rclcpp::ParameterValue(""));
      node->get_parameter(plugin + ".plugin", plugin_type);
      cx::ClipsFeature::Ptr feature =
          pg_loader_.createUniqueInstance(plugin_type);

      feature->initialize(parent_, plugin);

      RCLCPP_INFO(logger_, "Created feature: %s of type %s on activation",
                  plugin.c_str(), plugin_type.c_str());
      // Insert loaded feature to the features map
      features_.insert({plugin, std::move(feature)});
      success = features_[plugin]->clips_env_init(env);
    }
    if (success) {
      loaded_plugins_[env_name].push_back(plugin);
    }
  }
}

void ClipsFeatureManager::deactivate() {
  {
    std::scoped_lock env_lock(*envs_.get_mutex_instance());
    for (auto &env : *envs_.get_obj().get()) {
      deactivate_env(env.first, env.second);
    }
  }
  std::vector<std::string> loaded_plugin_types;
  std::string feature_plugin;
  for (const auto &f : features_) {
    auto node = parent_.lock();
    node->get_parameter(f.first + ".plugin", feature_plugin);
    loaded_plugin_types.push_back(feature_plugin);
    f.second->finalize();
  }
  features_.clear();
  for (const auto &f2 : loaded_plugin_types) {
    pg_loader_.unloadLibraryForClass(f2);
  }
  load_plugin_service_.reset();
  unload_plugin_service_.reset();
  list_plugin_service_.reset();
}

void ClipsFeatureManager::deactivate_env(
    const std::string &env_name, LockSharedPtr<clips::Environment> &env) {
  for (const auto &plugin :
       std::ranges::reverse_view(loaded_plugins_[env_name])) {
    features_[plugin]->clips_env_destroyed(env);
    RCLCPP_INFO(logger_, "[%s] Deactivated!", plugin.c_str());
  }
}

void ClipsFeatureManager::load_plugin_cb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<cx_msgs::srv::LoadCLIPSPlugin::Request> request,
    const std::shared_ptr<cx_msgs::srv::LoadCLIPSPlugin::Response> response) {
  (void)request_header; // ignoring request id
  std::string env_name = request->env_name;
  std::string plugin_name = request->plugin_name;
  if (envs_->contains(env_name)) {
    LockSharedPtr<clips::Environment> &clips = envs_->at(env_name);
    if (features_.contains(plugin_name)) {
      bool success = features_[plugin_name]->clips_env_init(clips);
      response->success = success;
      if (!success) {
        features_[plugin_name]->clips_env_destroyed(clips);
        response->error = "error while loading plugin";
      }
    } else {
      response->success = false;
      response->error = "unknown plugin";
      return;
    }
  } else {
    response->success = false;
    response->error = "unknown environment";
    return;
  }
}

void ClipsFeatureManager::unload_plugin_cb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<cx_msgs::srv::UnloadCLIPSPlugin::Request> request,
    const std::shared_ptr<cx_msgs::srv::UnloadCLIPSPlugin::Response> response) {
  (void)request_header; // ignoring request id
  std::string env_name = request->env_name;
  std::string plugin_name = request->plugin_name;
  if (envs_->contains(env_name)) {
    LockSharedPtr<clips::Environment> &clips = envs_->at(env_name);
    if (features_.contains(plugin_name)) {
      bool success = features_[plugin_name]->clips_env_destroyed(clips);
      response->success = success;
      if (!success) {
        response->error = "error while unloading plugin";
      }
    } else {
      response->success = false;
      response->error = "unknown plugin";
      return;
    }
  } else {
    response->success = false;
    response->error = "unknown environment";
    return;
  }
}

void ClipsFeatureManager::list_plugin_cb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<cx_msgs::srv::ListCLIPSPlugin::Request> request,
    const std::shared_ptr<cx_msgs::srv::ListCLIPSPlugin::Response> response) {
  (void)request_header; // ignoring request id
  std::string env_name = request->env_name;
  if (env_name == "") {
    response->success = true;
    std::vector<std::string> plugins;
    response->plugins = plugins;
    return;
  }
  if (envs_->contains(env_name)) {
    response->success = true;
    std::vector<std::string> plugins;
    response->plugins = plugins;
    return;
  } else {
    response->success = false;
    response->error = "unknown environment";
    return;
  }
}

} // namespace cx
