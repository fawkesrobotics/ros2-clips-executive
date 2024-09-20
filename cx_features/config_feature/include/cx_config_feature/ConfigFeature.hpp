// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  ConfigFeature.hpp
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

#ifndef CX_FEATURES__CONFIGFEATURE_HPP_
#define CX_FEATURES__CONFIGFEATURE_HPP_

#include <map>
#include <memory>
#include <string>

#include "cx_feature/clips_feature.hpp"
#include "cx_utils/LockSharedPtr.hpp"
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

class ConfigFeature : public ClipsFeature {
public:
  ConfigFeature(std::string agent_dir);
  ~ConfigFeature();

  void initialize(const std::string &feature_name) override;

  bool clips_context_init(const std::string &env_name,
                          LockSharedPtr<clips::Environment> &clips) override;
  bool clips_context_destroyed(const std::string &env_name) override;

  std::string getFeatureName() const;

private:
  void clips_config_load(clips::Environment *env, const std::string &file,
                         const std::string &cfg_prefix);

  std::string getScalarType(const YAML::Node &input_node);

  void iterateThroughYamlRecuresively(const YAML::Node &current_level_node,
                                      const std::string &logger_name,
                                      std::string cfg_prefix,
                                      clips::Environment *env);

  void sequenceIterator(const YAML::Node &input_node,
                        const std::string &logger_name, std::string &cfg_prefix,
                        clips::Environment *env);

private:
  std::map<std::string, LockSharedPtr<clips::Environment>> envs_;

  std::string agent_dir_;
};

} // namespace cx

#endif // !CX_FEATURES__CONFIGFEATURE_HPP_
