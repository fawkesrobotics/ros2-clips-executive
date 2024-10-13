// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  pddl_parser_feature.cpp
 *
 *  Created: 15 September 2021
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

#include <clips_pddl_parser/clips_pddl_parser.h>

#include "cx_feature/clips_feature.hpp"
#include "cx_pddl_parser_feature/pddl_parser_feature.hpp"
#include "cx_utils/LockSharedPtr.hpp"

#include <cx_utils/clips_env_context.hpp>

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

namespace cx {

PddlParserFeature::PddlParserFeature() {}
PddlParserFeature::~PddlParserFeature() {}

bool PddlParserFeature::clips_env_init(LockSharedPtr<clips::Environment> &env) {
  RCLCPP_DEBUG(rclcpp::get_logger(feature_name_), "Initializing feature %s",
               feature_name_.c_str());
  auto context = CLIPSEnvContext::get_context(env.get_obj().get());
  std::string env_name = context->env_name_;

  pddl_parsers_[env_name] =
      std::make_unique<clips_pddl_parser::ClipsPddlParser>(
          env.get_obj().get(), *(env.get_mutex_instance()), false);
  return true;
}

bool PddlParserFeature::clips_env_destroyed(
    LockSharedPtr<clips::Environment> &env) {
  auto context = CLIPSEnvContext::get_context(env.get_obj().get());
  std::string env_name = context->env_name_;

  RCLCPP_DEBUG(rclcpp::get_logger(feature_name_), "Destroying clips context!");

  pddl_parsers_.erase(env_name);
  return true;
}
} // namespace cx
PLUGINLIB_EXPORT_CLASS(cx::PddlParserFeature, cx::ClipsFeature)
