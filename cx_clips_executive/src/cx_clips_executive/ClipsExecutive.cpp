/***************************************************************************
 *  ClipsFeature.cpp
 *
 *  Created: 14 July 2021
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

#include <memory>
#include <string>

#include <yaml-cpp/yaml.h>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "cx_clips/CLIPSEnvManagerClient.hpp"
#include "cx_clips_executive/ClipsExecutive.hpp"
#include "cx_utils/params_utils.hpp"

using namespace std::chrono_literals;

namespace cx {

ClipsExecutive::ClipsExecutive()
    : rclcpp_lifecycle::LifecycleNode("clips_executive"), refresh_rate_{10} {

  RCLCPP_INFO(get_logger(), "Initialising [%s]...", get_name());
  cfg_assert_time_each_cycle_ = true;
  declare_parameter<std::vector<std::string>>("clips-dirs", clips_dirs);
  declare_parameter<std::string>("spec", "");
  declare_parameter<bool>("assert-time-each-loop", cfg_assert_time_each_cycle_);
  declare_parameter<int>("refresh-rate", refresh_rate_);
}

// ClipsExecutive::~ClipsExecutive() {}
void ClipsExecutive::pre_configure(
    std::shared_ptr<cx::CLIPSEnvManagerNode> &manager_node) {
  clips_env_manager_node_ = manager_node;
}

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn
ClipsExecutive::on_configure(const rclcpp_lifecycle::State &state) {
  (void)state; // ignoring unused parameter
  RCLCPP_INFO(get_logger(), "Configuring [%s]...", get_name());

  env_manager_client_ =
      std::make_shared<cx::CLIPSEnvManagerClient>("clips_manager_client_cx");
  clips_agenda_refresh_pub_ = create_publisher<std_msgs::msg::Empty>(
      "clips_executive/refresh_agenda", rclcpp::QoS(100).reliable());

  std::string cx_bringup_dir;
  std::string cx_features_dir;
  try {
    cx_bringup_dir =
        std::move(ament_index_cpp::get_package_share_directory("cx_bringup"));
    clips_executive_share_dir_ = std::move(
        ament_index_cpp::get_package_share_directory("cx_clips_executive"));
    cx_features_dir =
        std::move(ament_index_cpp::get_package_share_directory("cx_features"));
  } catch (const std::exception &e) {
    RCLCPP_ERROR(
        get_logger(),
        "Exception when getting bringup/clips_executive shared directory!");
    std::cerr << e.what() << '\n';
    return CallbackReturn::FAILURE;
  }

  if (!(get_parameter("clips-dirs", clips_dirs))) {
    RCLCPP_ERROR(
        get_logger(),
        "Couldnt get parameter /clips_executive/clips-dirs, aborting...");
    return CallbackReturn::FAILURE;
  }

  for (size_t i = 0; i < clips_dirs.size(); ++i) {
    if (clips_dirs[i][clips_dirs[i].size() - 1] != '/') {
      clips_dirs[i] += "/";
    }
    RCLCPP_INFO(get_logger(), "CLIPS_DIR: %s", clips_dirs[i].c_str());
  }
  // Default Clips files directory
  clips_dirs.insert(clips_dirs.begin(), clips_executive_share_dir_ + "/clips/");
  clips_dirs.insert(clips_dirs.begin(), cx_features_dir + "/clips/");

  std::string cfg_spec;
  get_parameter("spec", cfg_spec);
  if (cfg_spec == "") {
    RCLCPP_ERROR(get_logger(),
                 "Couldnt get parameter /clips_executive/spec, aborting...");
    return CallbackReturn::FAILURE;
  }

  get_parameter("assert-time-each-loop", cfg_assert_time_each_cycle_);
  get_parameter("refresh-rate", refresh_rate_);
  double rate = 1.0 / refresh_rate_;
  // Sets the time between each clips agenda refresh in ns
  publish_rate_ =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(rate));
  RCLCPP_INFO(
      get_logger(), "Publishing rate set to: %ldns",
      std::chrono::duration_cast<std::chrono::nanoseconds>(publish_rate_)
          .count());

  std::string action_mapping_cfgpath = "specs." + cfg_spec + ".action-mapping";

  std::map<std::string, std::string> action_mapping{};

  try {
    YAML::Node config = YAML::LoadFile(
        std::move(cx_bringup_dir + "/params/clips_executive.yaml"));
    iterateThroughYamlRecuresively(config["clips_executive"], "action-mapping",
                                   "", cfg_spec, action_mapping);

  } catch (const std::exception &e) {
    RCLCPP_INFO(get_logger(), "Error loading clips_executive config file!");

    std::cerr << e.what() << '\n';
    return CallbackReturn::FAILURE;
  }

  if (action_mapping.begin() == action_mapping.end()) {
    RCLCPP_WARN(get_logger(), "No action mapping provided!");
  }
  // Print all actions mappings
  for (const auto &action : action_mapping) {
    RCLCPP_INFO(get_logger(), "Adding action mapping %s->%s",
                action.first.c_str(), action.second.c_str());
  }
  action_skill_mapping_ =
      std::make_shared<cx::ActionSkillMapping>(action_mapping);

  env_manager_client_->createNewClipsEnvironment("executive", "executive");
  RCLCPP_INFO(get_logger(), "Configured [%s]!", get_name());
  return CallbackReturn::SUCCESS;
}

CallbackReturn
ClipsExecutive::on_activate(const rclcpp_lifecycle::State &state) {
  (void)state; // ignoring unused parameter
  RCLCPP_INFO(get_logger(), "Activating [%s]...", get_name());
  clips_agenda_refresh_pub_->on_activate();
  // Creating the Clips Executive environment in configure works async  --> busy
  // waiting to assure creation
  auto start_time = now();
  auto req_time = (now() - start_time).seconds();
  while (clips_env_manager_node_->envs_.find("executive") ==
         clips_env_manager_node_->envs_.end()) {
    req_time = (now() - start_time).seconds();
    if (req_time > 3.0) {
      RCLCPP_ERROR(get_logger(),
                   "Clips Executive Environment creation failed, aborting...");
      return CallbackReturn::FAILURE;
    }
  }
  clips_ = clips_env_manager_node_->envs_["executive"].env;

  clips_->reset();
  // TODO: REMOVE LATER!
  clips_->watch("all");

  std::lock_guard<std::mutex> guard(*(clips_.get_mutex_instance()));

  std::string cx_bringup_dir;
  std::string cx_features_dir;
  try {
    cx_bringup_dir =
        std::move(ament_index_cpp::get_package_share_directory("cx_bringup"));
    clips_->evaluate(std::string("(path-add-subst \"@CONFDIR@\" \"") +
                     cx_bringup_dir + "\")");
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return CallbackReturn::FAILURE;
  }

  for (size_t i = 0; i < clips_dirs.size(); ++i) {
    clips_->evaluate("(path-add \"" + clips_dirs[i] + "\")");
  }

  clips_->evaluate("(ff-feature-request \"config_feature\")");

  clips_->add_function(
      "map-action-skill",
      sigc::slot<std::string, std::string, CLIPS::Values, CLIPS::Values>(
          sigc::mem_fun(*this, &ClipsExecutive::clips_map_skill)));

  clips_->evaluate("(ff-feature-request \"redefine_warning_feature\")");

  std::vector<std::string> files{
      clips_executive_share_dir_ + "/clips/saliences.clp",
      clips_executive_share_dir_ + "/clips/init.clp"};
  for (const auto &f : files) {
    if (!clips_->batch_evaluate(f)) {
      RCLCPP_ERROR(get_logger(),
                   "Failed to initialize CLIPS environment, "
                   "batch file '%s' failed!, aborting...",
                   f.c_str());
      return CallbackReturn::FAILURE;
    }
  }

  clips_->assert_fact("(executive-init)");
  clips_->refresh_agenda();
  clips_->run();

  // Verify that initialization did not fail (yet)
  {
    CLIPS::Fact::pointer fact = clips_->get_facts();
    while (fact) {
      CLIPS::Template::pointer tmpl = fact->get_template();
      if (tmpl->name() == "executive-init-stage") {
        CLIPS::Values v = fact->slot_value("");
        if (v.size() > 0 && v[0].as_string() == "FAILED") {
          RCLCPP_ERROR(get_logger(), "CLIPS Executive initialization failed");
          return CallbackReturn::FAILURE;
        }
      }
      fact = fact->next();
    }
  }
  RCLCPP_INFO(get_logger(), "CLIPS Executive was inistialised!");

  agenda_refresh_timer_ = create_wall_timer(publish_rate_, [this]() {
    std::lock_guard<std::mutex> guard(*(clips_.get_mutex_instance()));

    if (cfg_assert_time_each_cycle_) {
      clips_->assert_fact("(time (now))");
    }

    clips_->refresh_agenda();
    clips_->run();
    clips_agenda_refresh_pub_->publish(std_msgs::msg::Empty());
  });
  RCLCPP_INFO(get_logger(), "Activated [%s]!", get_name());
  return CallbackReturn::SUCCESS;
}

CallbackReturn
ClipsExecutive::on_deactivate(const rclcpp_lifecycle::State &state) {
  (void)state; // ignoring unused parameter
  RCLCPP_INFO(get_logger(), "[%s] Deactivating...", get_name());
  clips_agenda_refresh_pub_->on_deactivate();
  clips_->assert_fact("(executive-finalize)");
  clips_->refresh_agenda();
  clips_->run();
  RCLCPP_INFO(get_logger(), "[%s] Deactivated", get_name());

  return CallbackReturn::SUCCESS;
}

std::string ClipsExecutive::clips_map_skill(std::string action_name,
                                            CLIPS::Values param_names,
                                            CLIPS::Values param_values) {
  if (!action_skill_mapping_) {
    RCLCPP_ERROR(get_logger(), "No action mapping has been loaded");
    return "";
  }
  if (action_name == "") {
    RCLCPP_WARN(get_logger(), "Failed to map, action name is empty");
    return "";
  }
  if (!action_skill_mapping_->has_mapping(action_name)) {
    RCLCPP_WARN(get_logger(), "No mapping for action '%s' known",
                action_name.c_str());
    return "";
  }
  if (param_names.size() != param_values.size()) {
    RCLCPP_WARN(get_logger(),
                "Number of parameter names and values "
                "do not match for action '%s'",
                action_name.c_str());
    return "";
  }
  std::map<std::string, std::string> param_map;
  for (size_t i = 0; i < param_names.size(); ++i) {
    if (param_names[i].type() != CLIPS::TYPE_SYMBOL &&
        param_names[i].type() != CLIPS::TYPE_STRING) {
      RCLCPP_ERROR(get_logger(), "Param for '%s' is not a string or symbol",
                   action_name.c_str());
      return "";
    }
    switch (param_values[i].type()) {
    case CLIPS::TYPE_FLOAT:
      param_map[param_names[i].as_string()] =
          std::to_string(param_values[i].as_float());
      break;
    case CLIPS::TYPE_INTEGER:
      param_map[param_names[i].as_string()] =
          std::to_string(param_values[i].as_integer());
      break;
    case CLIPS::TYPE_SYMBOL:
    case CLIPS::TYPE_STRING:
      param_map[param_names[i].as_string()] = param_values[i].as_string();
      break;
    default:
      RCLCPP_ERROR(get_logger(), "Param '%s' for action '%s' of invalid type",
                   param_names[i].as_string().c_str(), action_name.c_str());
      break;
    }
  }

  std::multimap<std::string, std::string> messages;
  std::string rv =
      action_skill_mapping_->map_skill(action_name, param_map, messages);
  for (auto &m : messages) {
    if (m.first == "WARN") {
      RCLCPP_WARN(get_logger(), "%s", m.second.c_str());
    } else if (m.first == "ERROR") {
      RCLCPP_ERROR(get_logger(), "%s", m.second.c_str());
    } else if (m.first == "DEBUG") {
      RCLCPP_DEBUG(get_logger(), "%s", m.second.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "%s", m.second.c_str());
    }
  }
  return rv;
}

void ClipsExecutive::iterateThroughYamlRecuresively(
    const YAML::Node &current_level_node, const std::string &node_to_search,
    const std::string &parent_node_name, const std::string &cfg_prefix,
    std::map<std::string, std::string> &output_map) {
  for (const auto &item : current_level_node) {
    switch (item.second.Type()) {

    case YAML::NodeType::Undefined: {
      RCLCPP_ERROR(get_logger(), "Undefined YAML KEY");
      break;
    }
    case YAML::NodeType::Null: {
      RCLCPP_ERROR(get_logger(), "NULL YAML KEY");
      break;
    }
    case YAML::NodeType::Scalar: {
      // skip
      break;
    }
    case YAML::NodeType::Sequence: {
      // skip
      break;
    }
    // If it is a MapNode
    case YAML::NodeType::Map: {
      if (item.first.as<std::string>() == node_to_search &&
          parent_node_name == cfg_prefix) {

        for (const auto &map_values : current_level_node[item.first]) {
          output_map.insert({map_values.first.as<std::string>(),
                             map_values.second.as<std::string>()});
          RCLCPP_INFO(get_logger(), "Key %s <------> Value: %s",
                      map_values.first.as<std::string>().c_str(),
                      output_map[map_values.first.as<std::string>()].c_str());
        }
      } else {
        iterateThroughYamlRecuresively(
            current_level_node[item.first], node_to_search,
            item.first.as<std::string>(), cfg_prefix, output_map);
      }
      break;
    }
    }
  }
}

} // namespace cx