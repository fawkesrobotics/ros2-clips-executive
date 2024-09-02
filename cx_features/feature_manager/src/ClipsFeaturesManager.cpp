// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  ClipsFeaturesManager.cpp
 *
 *  Created: 25 June 2021
 *  Copyright  2021  Ivaylo Doychev
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <map>
#include <memory>
#include <string>
#include <utility>

#include "cx_feature_manager/ClipsFeaturesManager.hpp"

#include "cx_core/ClipsFeature.hpp"
#include "cx_msgs/srv/clips_feature_context.hpp"
#include "cx_utils/params_utils.hpp"

#include "lifecycle_msgs/msg/state.hpp"

using namespace std::placeholders;

namespace cx {

ClipsFeaturesManager::ClipsFeaturesManager()
    : rclcpp_lifecycle::LifecycleNode(
          "clips_features_manager",
          rclcpp::NodeOptions().allow_undeclared_parameters(true)),
      env_manager_client_{std::make_shared<cx::CLIPSEnvManagerClient>(
          "clips_manager_client_fm")},
      pg_loader_("cx_core", "cx::ClipsFeature"), default_ids_{},
      default_types_{} {

  declare_parameter("clips_features_list", default_ids_);
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
  (void)state; // ignoring unused parameter
  RCLCPP_INFO(get_logger(), "Configuring [%s]...", get_name());
  declare_parameter<std::string>(
      "agent_dir", ament_index_cpp::get_package_share_directory("cx_bringup"));

  auto node = shared_from_this();

  feature_destroy_context_service_ =
      create_service<cx_msgs::srv::ClipsFeatureContext>(
          "clips_features_manager/destroy_feature_context",
          std::bind(&ClipsFeaturesManager::feature_destroy_context_callback,
                    this, _1, _2, _3));

  // WE FIRST ADD ALL DEFAULT FEATURES
  addGeneralFeatures();

  // Load all registered features in the cx_params.yml file
  get_parameter("clips_features_list", features_ids_);
  RCLCPP_INFO(get_logger(), "Detect features:");
  for (const auto &feat : features_ids_) {
    RCLCPP_INFO(get_logger(), "Detected [ %s ]", feat.c_str());
  }

  if (!features_ids_.empty()) {
    features_types_.resize(features_ids_.size());

    for (size_t i = 0; i < features_types_.size(); i++) {
      try {
        const std::string feat_name = features_ids_[i];

        // declare the parameters define in the list for each feature
        std::map<std::string, rclcpp::Parameter> feature_param_map{};
        feature_param_map["agent_dir"] = get_parameter("agent_dir");

        declare_parameter("clips_features." + feat_name + ".feature_parameters",
                          std::vector<std::string>());
        std::vector<std::string> feature_parameters =
            get_parameter("clips_features." + feat_name + ".feature_parameters")
                .as_string_array();

        for (const std::string &feat_param : feature_parameters) {
          // declare the parameter
          declare_parameter(
              "clips_features." + feat_name + "." + feat_param + ".type", "");
          std::string param_type = get_parameter("clips_features." + feat_name +
                                                 "." + feat_param + ".type")
                                       .as_string();

          if (param_type == "string") {
            declare_parameter("clips_features." + feat_name + "." + feat_param +
                                  ".value",
                              rclcpp::PARAMETER_STRING);
          }
          if (param_type == "integer") {
            declare_parameter("clips_features." + feat_name + "." + feat_param +
                                  ".value",
                              rclcpp::PARAMETER_INTEGER);
          }
          if (param_type == "double") {
            declare_parameter("clips_features." + feat_name + "." + feat_param +
                                  ".value",
                              rclcpp::PARAMETER_DOUBLE);
          }
          if (param_type == "bool") {
            declare_parameter("clips_features." + feat_name + "." + feat_param +
                                  ".value",
                              rclcpp::PARAMETER_BOOL);
          }
          if (param_type == "byte-array") {
            declare_parameter("clips_features." + feat_name + "." + feat_param +
                                  ".value",
                              rclcpp::PARAMETER_BOOL_ARRAY);
          }
          if (param_type == "string-array") {
            declare_parameter("clips_features." + feat_name + "." + feat_param +
                                  ".value",
                              rclcpp::PARAMETER_STRING_ARRAY);
          }
          if (param_type == "integer-array") {
            declare_parameter("clips_features." + feat_name + "." + feat_param +
                                  ".value",
                              rclcpp::PARAMETER_INTEGER_ARRAY);
          }
          if (param_type == "double-array") {
            declare_parameter("clips_features." + feat_name + "." + feat_param +
                                  ".value",
                              rclcpp::PARAMETER_DOUBLE_ARRAY);
          }
          if (param_type == "bool-array") {
            declare_parameter("clips_features." + feat_name + "." + feat_param +
                                  ".value",
                              rclcpp::PARAMETER_BOOL_ARRAY);
          }
          feature_param_map[feat_param] = get_parameter(
              "clips_features." + feat_name + "." + feat_param + ".value");
        }

        features_types_[i] =
            cx::get_plugin_type_param(node, "clips_features." + feat_name);

        cx::ClipsFeature::Ptr feature =
            pg_loader_.createUniqueInstance(features_types_[i]);

        // Call the feature initialisation
        if (feature_parameters.size() > 0) {
          feature->initialise(feat_name, feature_param_map);
        } else {
          feature->initialise(feat_name);
        }

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
  env_manager_client_->addFeatures(feature_names_vector_);

  RCLCPP_INFO(get_logger(), "Configured [%s]!", get_name());
  return CallbackReturn::SUCCESS;
}

CallbackReturn
ClipsFeaturesManager::on_activate(const rclcpp_lifecycle::State &state) {
  (void)state; // ignoring unused parameter
  RCLCPP_INFO(get_logger(), "Activating [%s]...", get_name());
  // Add ff-request feature, as it is implemented in features manager
  for (auto &envd : clips_env_manager_node_->envs_) {
    // std::lock_guard<std::mutex>
    // guard(*(envd.second.env.get_mutex_instance()));
    clips::AddUDF(
        envd.second.env.get_obj().get(), "ff-feature-request", "b", 1, 1, ";sy",
        [](clips::Environment *env, clips::UDFContext *udfc,
           clips::UDFValue *out) {
          ClipsFeaturesManager *instance =
              static_cast<ClipsFeaturesManager *>(udfc->context);

          clips::UDFValue feature_name;
          using namespace clips;
          clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &feature_name);

          // Call the member function and return the result
          instance->clips_request_feature(env, out,
                                          feature_name.lexemeValue->contents);
        },
        "clips_request_feature", this);
  }

  RCLCPP_INFO(get_logger(), "Activated [%s]!", get_name());
  return CallbackReturn::SUCCESS;
}

CallbackReturn
ClipsFeaturesManager::on_deactivate(const rclcpp_lifecycle::State &state) {
  (void)state; // ignoring unused parameter
  RCLCPP_INFO(get_logger(), "[%s] Deactivated!", get_name());
  for (auto &envd : clips_env_manager_node_->envs_) {
    // std::lock_guard<std::mutex>
    // guard(*(envd.second.env.get_mutex_instance()));
    clips::RemoveUDF(envd.second.env.get_obj().get(), "ff-feature-request");
  }
  features_.clear();

  return CallbackReturn::SUCCESS;
}

void ClipsFeaturesManager::clips_request_feature(
    clips::Environment *env, clips::UDFValue *out,
    const std::string &feature_name) {
  bool found_env = false;
  std::string env_name;

  for (auto &entry : clips_env_manager_node_->envs_) {
    if (entry.second.env.get_obj().get() == env) {
      env_name = entry.first;
      found_env = true;
      break;
    }
  }
  if (!found_env) {
    RCLCPP_ERROR(get_logger(),
                 "Unable to determine environment from raw pointer");
    out->lexemeValue = clips::CreateBoolean(env, false);
    return;
  }
  bool rv = true;

  RCLCPP_INFO(get_logger(), "Environment %s requests feature %s",
              env_name.c_str(), feature_name.c_str());
  // Check if the feature is available
  if (clips_env_manager_node_->features_set.find(feature_name) ==
      clips_env_manager_node_->features_set.end()) {
    RCLCPP_WARN(get_logger(), "Environment %s requested unavailable feature %s",
                env_name.c_str(), feature_name.c_str());
    out->lexemeValue = clips::CreateBoolean(env, false);
    return;
  }

  CLIPSEnvManagerNode::ClipsEnvData &envd =
      clips_env_manager_node_->envs_[env_name];
  // Check if features was already requested
  if (std::binary_search(envd.req_feat.begin(), envd.req_feat.end(),
                         feature_name)) {
    RCLCPP_WARN(get_logger(), "Environment %s requested feature %s *again*",
                env_name.c_str(), feature_name.c_str());
    out->lexemeValue = clips::CreateBoolean(env, true);
    return;
  }
  feature_init_context(env_name, feature_name);
  envd.req_feat.push_back(feature_name);
  envd.req_feat.sort();

  // deffacts to sruvive reset
  std::string deffacts = "(deffacts ff-features-loaded";

  for (const auto &feat : envd.req_feat) {
    deffacts += " (ff-feature-loaded " + feat + ")";
  }

  deffacts += ")";

  clips::AssertString(
      envd.env.get_obj().get(),
      std::format("(ff-feature-loaded {})", feature_name.c_str()).c_str());

  if (clips::Build(envd.env.get_obj().get(), deffacts.c_str()) !=
      clips::BuildError::BE_NO_ERROR) {
    RCLCPP_WARN(get_logger(),
                "Failed to build deffacts ff-features-loaded for %s",
                env_name.c_str());
    rv = false;
  }
  out->lexemeValue = clips::CreateBoolean(env, rv);
}

void ClipsFeaturesManager::feature_init_context(
    const std::string &env_name, const std::string &feature_name) {

  LockSharedPtr<clips::Environment> &clips =
      clips_env_manager_node_->envs_[env_name].env;
  // std::lock_guard<std::mutex> guard(*(clips.get_mutex_instance()));

  if (features_.find(feature_name) != features_.end()) {
    bool success = features_[feature_name]->clips_context_init(env_name, clips);
    if (!success) {
      RCLCPP_ERROR(get_logger(), "Error by context initialisation: feature %s",
                   feature_name.c_str());
    }
  } else {
    RCLCPP_ERROR(get_logger(), "%s is not registered!", feature_name.c_str());
  }
}

void ClipsFeaturesManager::feature_destroy_context_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<cx_msgs::srv::ClipsFeatureContext::Request> request,
    const std::shared_ptr<cx_msgs::srv::ClipsFeatureContext::Response>
        response) {
  (void)request_header; // ignoring request id
  const std::string &env_name = request->env_name;
  const std::string &feature_name = request->feature_name;
  RCLCPP_INFO(get_logger(), "IN DESTROY CONTEXT CALLBACK FOR FEATURE %s",
              feature_name.c_str());

  if (features_.find(feature_name) != features_.end()) {
    bool success = features_[feature_name]->clips_context_destroyed(env_name);
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

void ClipsFeaturesManager::addGeneralFeatures() {
  std::string agent_dir;
  get_parameter("agent_dir", agent_dir);
  auto configFeature = std::make_shared<cx::ConfigFeature>(agent_dir);
  configFeature->initialise("config_feature");
  RCLCPP_INFO(get_logger(), "Created feature config_feature");

  features_.insert({"config_feature", configFeature});
}
} // namespace cx
