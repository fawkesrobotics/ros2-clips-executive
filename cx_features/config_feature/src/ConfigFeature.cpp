// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  ConfigFeature.cpp
 *
 *  Created: 04 July 2021
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

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include "cx_config_feature/ConfigFeature.hpp"

namespace cx {
ConfigFeature::ConfigFeature(std::string agent_dir) : agent_dir_(agent_dir) {}

ConfigFeature::~ConfigFeature() {}

std::string ConfigFeature::getFeatureName() const { return clips_feature_name; }

void ConfigFeature::initialise(const std::string &feature_name) {
  clips_feature_name = feature_name;
  // clips_config_load("clips-executive", "/clips-executive");
}

bool ConfigFeature::clips_context_init(
    const std::string &env_name, LockSharedPtr<clips::Environment> &clips) {
  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Initialising context for feature %s",
              clips_feature_name.c_str());

  envs_[env_name] = clips;
  clips::Eval(clips.get_obj().get(), "(path-load \"ff-config.clp\")", NULL);
  clips::AddUDF(
      clips.get_obj().get(), "config-load", "v", 2, 2, "sy;sy",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        ConfigFeature *instance = static_cast<ConfigFeature *>(udfc->context);
        using namespace clips;
        clips::UDFValue file;
        clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &file);
        clips::UDFValue cfg_prefix;
        clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &cfg_prefix);
        instance->clips_config_load(env, file.lexemeValue->contents,
                                    cfg_prefix.lexemeValue->contents);
      },
      "clips_config_load", this);

  clips::RefreshAllAgendas(clips.get_obj().get());
  clips::Run(clips.get_obj().get(), -1);

  return true;
}

bool ConfigFeature::clips_context_destroyed(const std::string &env_name) {

  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Destroying clips context for feature %s!",
              clips_feature_name.c_str());
  clips::RemoveUDF(envs_[env_name].get_obj().get(), "config-load");
  envs_.erase(env_name);
  return true;
}
void ConfigFeature::clips_config_load(clips::Environment *env,
                                      const std::string &file,
                                      const std::string &cfg_prefix) {
  const std::string name = "ClipsConfig";
  const std::string cfg_main_node = cfg_prefix.substr(1, cfg_prefix.size() - 1);

  try {
    YAML::Node config = YAML::LoadFile(file);
    // std::lock_guard<std::mutex>
    // guard(*(envs_[env_name].get_mutex_instance()));

    iterateThroughYamlRecuresively(config[cfg_main_node], name, cfg_prefix,
                                   env);

  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(name), e.what());
    RCLCPP_WARN(rclcpp::get_logger(name), "Abroting config loading...");
  }
}

void ConfigFeature::iterateThroughYamlRecuresively(
    const YAML::Node &current_level_node, const std::string &logger_name,
    std::string cfg_prefix, clips::Environment *env) {

  std::string config_path = cfg_prefix;
  // RCLCPP_INFO(rclcpp::get_logger(logger_name), "CFG PATH: %s",
  //             config_path.c_str());

  for (const auto &item : current_level_node) {
    std::string type = "";
    std::string path = "";

    switch (item.second.Type()) {
    // If it is a NullNode
    case YAML::NodeType::Undefined: {
      RCLCPP_ERROR(rclcpp::get_logger(logger_name), "Undefined YAML KEY");
      break;
    }
    case YAML::NodeType::Null: {
      RCLCPP_ERROR(rclcpp::get_logger(logger_name), "NULL YAML KEY");
      break;
    }

    // If it is a ScalarNode -> Single key/value pair
    case YAML::NodeType::Scalar: {

      type = std::move(getScalarType(item.second));
      path = config_path + "/" + item.first.as<std::string>();

      if (type == "STRING") {
        std::stringstream escaped_quotes;
        escaped_quotes << std::quoted(item.second.as<std::string>());
        // RCLCPP_INFO(rclcpp::get_logger(logger_name),
        //             "(confval (path \"%s\") (type %s) (value %s))",
        //             path.c_str(), type.c_str(),
        //             escaped_quotes.str().c_str());

        clips::AssertString(
            env, std::format("(confval (path \"{}\") (type {}) (value {}))",
                             path.c_str(), type.c_str(),
                             escaped_quotes.str().c_str())
                     .c_str());
      } else {
        // RCLCPP_INFO(rclcpp::get_logger(logger_name),
        //             "(confval (path \"%s\") (type %s) (value %s))",
        //             path.c_str(), type.c_str(),
        //             item.second.as<std::string>().c_str());
        if (item.second.as<std::string>() == "true" ||
            item.second.as<std::string>() == "false") {

          std::string val = item.second.as<std::string>();
          std::transform(val.begin(), val.end(), val.begin(), ::toupper);
          clips::AssertString(
              env, std::format("(confval (path \"{}\") (type {}) (value {}))",
                               path.c_str(), type.c_str(), val.c_str())
                       .c_str());
        } else {

          clips::AssertString(
              env, std::format("(confval (path \"{}\") (type {}) (value {}))",
                               path.c_str(), type.c_str(),
                               item.second.as<std::string>().c_str())
                       .c_str());
        }
      }
      break;
    }

    // If it is a SequenceNode
    case YAML::NodeType::Sequence: {

      const YAML::Node &nested_node = item.second;
      path = config_path + "/" + item.first.as<std::string>();

      sequenceIterator(nested_node, logger_name, path, env);

      break;
    }
    // If it is a MapNode
    case YAML::NodeType::Map: {
      if (item.first.as<std::string>() != "ros__parameters") {
        path = config_path + "/" + item.first.as<std::string>();
      } else {
        path = config_path;
      }
      iterateThroughYamlRecuresively(current_level_node[item.first],
                                     logger_name, path, env);
      break;
    }
    }
  }
}

void ConfigFeature::sequenceIterator(const YAML::Node &input_node,
                                     const std::string &logger_name,
                                     std::string &cfg_prefix,
                                     clips::Environment *env) {

  size_t i = 0;
  std::string path;
  std::string list_values{};
  int sequenceIndex = 0;

  for (const auto &item2 : input_node) {
    if (item2) {
      if (item2.IsScalar()) {
        // Direct sequence of the form ["1", "2", ...]
        list_values = list_values + " " + "\"" + item2.as<std::string>() + "\"";
        i++;

        if (i == input_node.size()) {
          clips::AssertString(
              env, std::format("(confval (path \"{}\") (type STRING) "
                               "(is-list TRUE) (list-value {}))",
                               cfg_prefix.c_str(), list_values.c_str())
                       .c_str());
          // RCLCPP_INFO(rclcpp::get_logger(logger_name),
          //             "(confval (path \"%s\") (type STRING) "
          //             "(is-list TRUE) (list-value%s))",
          //             cfg_prefix.c_str(), list_values.c_str());
        }

        continue;
      }
      // item2 is a Map or Sequence

      for (const auto &item3 : item2) {
        // OF the FORM
        //   - name: domain,
        //     file: test-scenario/test-domain.clp
        if (item3.second.IsScalar()) {

          path = cfg_prefix + "/" + std::move(std::to_string(sequenceIndex)) +
                 "/" + item3.first.as<std::string>();

          // path = path + std::move(std::to_string(sequenceIndex));

          clips::AssertString(
              env,
              std::format(
                  "(confval (path \"{}\") (is-list FALSE) (type {}) (value "
                  "\"{}\"))",
                  path.c_str(), std::move(getScalarType(item3.second)).c_str(),
                  item3.second.as<std::string>().c_str())
                  .c_str());

          // RCLCPP_INFO(rclcpp::get_logger(logger_name),
          //             "(confval (path \"%s\") (is-list FALSE) (type %s)
          //             (value "
          //             "\"%s\"))",
          //             path.c_str(),
          //             std::move(getScalarType(item3.second)).c_str(),
          //             item3.second.as<std::string>().c_str());

        } else if (item3.second.IsSequence()) {
          // OF the FORM
          //  action-execution:
          //    - skills-actions.clp
          //    - test-scenario-pddl/print-action.clp

          path = cfg_prefix + "/" + std::move(std::to_string(sequenceIndex)) +
                 "/" + item3.first.as<std::string>();
          // path = path + std::move(std::to_string(sequenceIndex));

          list_values = "";

          std::stringstream escaped_quotes;
          for (const auto &el : item3.second) {
            // if (list_values != "") {
            //   list_values = " " + el.as<std::string>();

            // } else {
            //   list_values = el.as<std::string>();
            // }
            escaped_quotes << std::quoted(el.as<std::string>());
          }

          clips::AssertString(
              env, std::format("(confval (path \"{}\") (type STRING) "
                               "(is-list TRUE) (list-value {}))",
                               path.c_str(), escaped_quotes.str().c_str())
                       .c_str());
          // RCLCPP_INFO(rclcpp::get_logger(logger_name),
          //             "(confval (path \"%s\") (type STRING) "
          //             "(is-list TRUE) (list-value%s))",
          //             path.c_str(), escaped_quotes.str().c_str());

        } else {
          RCLCPP_INFO(rclcpp::get_logger(logger_name), "MAP AGAIN");
          sequenceIterator(item3.second, logger_name, cfg_prefix, env);
        }
      }
      sequenceIndex++;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger(logger_name),
                   "Something went wrong while trying to iterate through the "
                   "yaml file!");
    }
  }
}

std::string ConfigFeature::getScalarType(const YAML::Node &input_node) {
  std::optional<bool> as_bool =
      YAML::as_if<bool, std::optional<bool>>(input_node)();
  if (as_bool) {
    return "BOOL";
  }

  std::optional<int> as_int =
      YAML::as_if<int, std::optional<int>>(input_node)();
  if (as_int) {
    return "INT";
  }

  std::optional<double> as_double =
      YAML::as_if<double, std::optional<double>>(input_node)();
  if (as_double) {
    return "FLOAT";
  }

  std::optional<std::string> as_string =
      YAML::as_if<std::string, std::optional<std::string>>(input_node)();
  if (as_string) {
    return "STRING";
  }
  return "UNKNOWN";
}

} // namespace cx
