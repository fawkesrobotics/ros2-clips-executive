// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

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
  declare_parameter<std::string>(
      "agent_dir", ament_index_cpp::get_package_share_directory("cx_bringup"));
  declare_parameter<bool>("assert_time_each_loop", cfg_assert_time_each_cycle_);
  declare_parameter<int>("refresh_rate", refresh_rate_);
  try {
    get_parameter("agent_dir", agent_dir_);
    declare_parameter("clips_executive_config",
                      agent_dir_ + "/params/clips_executive.yaml");
    get_parameter("clips_executive_config", clips_executive_config_);
    declare_parameter("clips_features_manager_config", clips_executive_config_);
    get_parameter("clips_features_manager_config",
                  clips_features_manager_config_);
    RCLCPP_INFO(get_logger(), "Load agent at [ %s ]", agent_dir_.c_str());
    RCLCPP_INFO(get_logger(), "with cx config [ %s ]",
                clips_executive_config_.c_str());
    RCLCPP_INFO(get_logger(), "and features config [%s]",
                clips_features_manager_config_.c_str());
    clips_executive_share_dir_ = std::move(
        ament_index_cpp::get_package_share_directory("cx_clips_executive"));
  } catch (const std::exception &e) {
    RCLCPP_ERROR(
        get_logger(),
        "Exception when getting bringup/clips_executive shared directory!");
    std::cerr << e.what() << '\n';
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
  clips_dirs.insert(clips_dirs.begin(), agent_dir_ + "/clips/");

  get_parameter("assert_time_each_loop", cfg_assert_time_each_cycle_);
  get_parameter("refresh_rate", refresh_rate_);
  double rate = 1.0 / refresh_rate_;
  // Sets the time between each clips agenda refresh in ns
  publish_rate_ =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(rate));
  RCLCPP_INFO(
      get_logger(), "Publishing rate set to: %ldns",
      std::chrono::duration_cast<std::chrono::nanoseconds>(publish_rate_)
          .count());

  std::string action_mapping_cfgpath = "action_mapping";

  std::map<std::string, std::string> action_mapping{};

  try {
    YAML::Node config = YAML::LoadFile(clips_executive_config_);
    action_mapping = get_action_mapping(config);

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

std::vector<std::string>
ClipsExecutive::multifield_to_string_vec(clips::Multifield *multi,
                                         const std::string &fun_name,
                                         clips::UDFContext *udfc) {
  std::vector<std::string> res;
  for (size_t i = 0; i < multi->length; i++) {
    switch (multi->contents[i].header->type) {
    case INTEGER_TYPE:
      res.push_back(std::to_string(multi->contents[i].integerValue->contents));
      break;
    case FLOAT_TYPE:
      res.push_back(std::to_string(multi->contents[i].floatValue->contents));
      break;
    case STRING_TYPE:
    case SYMBOL_TYPE:
      res.push_back(multi->contents[i].lexemeValue->contents);
      break;
    default:
      RCLCPP_ERROR(get_logger(),
                   "Unexpected Type %i of %li nth argument of UDF %s",
                   multi->contents[i].header->type, i, fun_name.c_str());
      clips::UDFThrowError(udfc);
    }
  }
  return res;
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

  clips::Reset(clips_.get_obj().get());

  std::lock_guard<std::mutex> guard(*(clips_.get_mutex_instance()));

  std::string cx_clips_executive_dir;
  try {
    cx_clips_executive_dir =
        ament_index_cpp::get_package_share_directory("cx_clips_executive");
    clips::Eval(
        clips_.get_obj().get(),
        (std::string("(path-add-subst \"@AGENT_DIR@\" \"") + agent_dir_ + "\")")
            .c_str(),
        NULL);
    clips::Eval(clips_.get_obj().get(),
                (std::string("(path-add-subst \"@CX_DIR@\" \"") +
                 cx_clips_executive_dir + "\")")
                    .c_str(),
                NULL);
    clips::Build(clips_.get_obj().get(), ("(defglobal ?*CX_CONFIG* = \"" +
                                          clips_executive_config_ + "\")")
                                             .c_str());
    clips::Build(clips_.get_obj().get(),
                 ("(defglobal ?*FEATURES_CONFIG* = \"" +
                  clips_features_manager_config_ + "\")")
                     .c_str());
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return CallbackReturn::FAILURE;
  }

  for (size_t i = 0; i < clips_dirs.size(); ++i) {
    clips::Eval(clips_.get_obj().get(),
                ("(path-add \"" + clips_dirs[i] + "\")").c_str(), NULL);
  }

  clips::Eval(clips_.get_obj().get(), "(ff-feature-request \"config_feature\")",
              NULL);

  clips::AddUDF(
      clips_.get_obj().get(), "map-action-skill", "sy", 3, 3, "*;s;m;m",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        ClipsExecutive *instance = static_cast<ClipsExecutive *>(udfc->context);
        clips::UDFValue action_name;
        clips::UDFValue action_param_names;
        clips::UDFValue action_param_values;
        using namespace clips;
        bool success =
            clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &action_name);
        success =
            success && clips::UDFNthArgument(udfc, 2, clips::MULTIFIELD_BIT,
                                             &action_param_names);
        success =
            success && clips::UDFNthArgument(udfc, 2, clips::MULTIFIELD_BIT,
                                             &action_param_values);
        if (!success) {
          RCLCPP_ERROR(instance->get_logger(),
                       "map-action-skill: Unexpected argument signature");
          clips::UDFThrowError(udfc);
        }
        std::vector<std::string> param_names_vec =
            instance->multifield_to_string_vec(
                action_param_names.multifieldValue,
                "map-action-skill[param_names]", udfc);
        std::vector<std::string> param_values_vec =
            instance->multifield_to_string_vec(
                action_param_values.multifieldValue,
                "map-action-skill[param_values]", udfc);
        std::string res =
            instance->clips_map_skill(action_name.lexemeValue->contents,
                                      param_names_vec, param_values_vec);
        out->lexemeValue = CreateString(env, res.c_str());
      },
      "clips_map_skill", this);

  std::vector<std::string> files{
      clips_executive_share_dir_ + "/clips/core/saliences.clp",
      clips_executive_share_dir_ + "/clips/core/init.clp"};
  for (const auto &f : files) {
    if (!clips::BatchStar(clips_.get_obj().get(), f.c_str())) {
      RCLCPP_ERROR(get_logger(),
                   "Failed to initialize CLIPS environment, "
                   "batch file '%s' failed!, aborting...",
                   f.c_str());
      return CallbackReturn::FAILURE;
    }
  }

  clips::AssertString(clips_.get_obj().get(), "(executive-init)");
  clips::RefreshAllAgendas(clips_.get_obj().get());
  clips::Run(clips_.get_obj().get(), -1);

  agenda_refresh_timer_ = create_wall_timer(publish_rate_, [this]() {
    std::lock_guard<std::mutex> guard(*(clips_.get_mutex_instance()));

    if (cfg_assert_time_each_cycle_) {
      clips::AssertString(clips_.get_obj().get(), "(time (now))");
    }

    clips::RefreshAllAgendas(clips_.get_obj().get());
    clips::Run(clips_.get_obj().get(), -1);
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
  clips::AssertString(clips_.get_obj().get(), "(executive-finalize)");
  clips::RefreshAllAgendas(clips_.get_obj().get());
  clips::Run(clips_.get_obj().get(), -1);
  clips::RemoveUDF(clips_.get_obj().get(), "map-action-skill");
  RCLCPP_INFO(get_logger(), "[%s] Deactivated", get_name());

  return CallbackReturn::SUCCESS;
}

std::string
ClipsExecutive::clips_map_skill(const std::string &action_name,
                                const std::vector<std::string> &param_names,
                                const std::vector<std::string> &param_values) {
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
    param_map[param_names[i]] = param_values[i];
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

// Search recursively from the given yaml node to a node, where the
// node[target_key] element exists
YAML::Node ClipsExecutive::get_node_from_key(const YAML::Node &node,
                                             const std::string &target_key) {
  if (!node) {
    return YAML::Node(YAML::NodeType::Undefined);
  }

  if (node.Type() == YAML::NodeType::Map) {
    for (const auto &entry : node) {
      const std::string &key = entry.first.as<std::string>();

      if (key == target_key) {
        return node;
      }

      // Recursively call for nested nodes
      auto result = get_node_from_key(entry.second, target_key);
      if (result.Type() != YAML::NodeType::Undefined) {
        return result;
      }
    }
  } else if (node.Type() == YAML::NodeType::Sequence) {
    // Iterate through sequence elements
    for (std::size_t i = 0; i < node.size(); ++i) {
      // Recursively call for each element in the sequence
      auto result = get_node_from_key(node[i], target_key);
      if (result.Type() != YAML::NodeType::Undefined) {
        return result;
      }
    }
  }
  return YAML::Node(YAML::NodeType::Undefined);
}

std::map<std::string, std::string>
ClipsExecutive::get_action_mapping(const YAML::Node &starting_node) {
  std::map<std::string, std::string> output_map;

  try {
    auto mapping_node = get_node_from_key(starting_node, "action_mapping");

    for (const auto &entry : mapping_node["action_mapping"]) {
      std::string name = entry.first.as<std::string>();
      std::string mapped_to;

      try {
        mapped_to = entry.second["mapped_to"].as<std::string>();
      } catch (const YAML::Exception &e) {
        RCLCPP_ERROR(get_logger(), "Error getting 'mapped_to' value of %s: %s",
                     name.c_str(), e.what());
        continue;
      }

      output_map[name] = mapped_to;
    }

    // Now the 'output_map' map contains the names and mapped-to values
    for (const auto &pair : output_map) {
      RCLCPP_INFO(get_logger(), "Action Mapping: Key %s <------> Value: %s",
                  pair.first.c_str(), pair.second.c_str());
    }

    return output_map;
  } catch (const YAML::Exception &e) {
    RCLCPP_WARN(get_logger(), "No action mapping found");
    return output_map;
  }
}

} // namespace cx
