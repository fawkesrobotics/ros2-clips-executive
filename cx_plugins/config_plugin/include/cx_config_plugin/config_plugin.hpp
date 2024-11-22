// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  config_plugin.hpp
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

#ifndef CX_PLUGINS__CONFIGPLUGIN_HPP_
#define CX_PLUGINS__CONFIGPLUGIN_HPP_

#include <map>
#include <memory>
#include <string>

#include "cx_plugin/clips_plugin.hpp"
#include "cx_utils/lock_shared_ptr.hpp"
#include <yaml-cpp/yaml.h>

#include "rcl_yaml_param_parser/parser.h"

// HACKY WAY TO DETERMINE SCALAR TYPE AS YAML-CPP DOESN'T OFFER THAT..
namespace YAML {
template <typename T> struct as_if<T, std::optional<T>> {
  explicit as_if(const Node &input_node) : node(input_node) {}
  const Node &node;

  const std::optional<T> operator()() const {
    std::optional<T> val;
    T t;
    if (node.m_pNode && convert<T>::decode(node, t))
      val = std::move(t);

    return val;
  }
};

// There is already a std::string partial specialisation, so we need a full
// specialisation here
template <> struct as_if<std::string, std::optional<std::string>> {
  explicit as_if(const Node &input_node) : node(input_node) {}
  const Node &node;

  const std::optional<std::string> operator()() const {
    std::optional<std::string> val;
    std::string t;
    if (node.m_pNode && convert<std::string>::decode(node, t))
      val = std::move(t);

    return val;
  }
};
} // namespace YAML

namespace cx {

class ConfigPlugin : public ClipsPlugin {
public:
  ConfigPlugin();
  ~ConfigPlugin();

  void initialize() override;

  bool clips_env_init(LockSharedPtr<clips::Environment> &env) override;
  bool clips_env_destroyed(LockSharedPtr<clips::Environment> &env) override;

private:
  void clips_config_load(clips::Environment *env, const std::string &file,
                         const std::string &cfg_prefix);

  std::string getScalarType(const YAML::Node &input_node);

  void iterateThroughYamlRecuresively(const YAML::Node &current_level_node,
                                      std::string cfg_prefix,
                                      clips::Environment *env);

  void sequenceIterator(const YAML::Node &input_node, std::string &cfg_prefix,
                        clips::Environment *env);

  std::unique_ptr<rclcpp::Logger> logger_;
};

} // namespace cx

#endif // !CX_PLUGINS__CONFIGPLUGIN_HPP_
