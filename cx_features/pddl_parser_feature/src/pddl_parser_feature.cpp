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

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

namespace cx {

PddlParserFeature::PddlParserFeature() {}
PddlParserFeature::~PddlParserFeature() {}

bool PddlParserFeature::clips_context_init(
    const std::string &env_name, LockSharedPtr<clips::Environment> &clips) {
  RCLCPP_DEBUG(rclcpp::get_logger(clips_feature_name_),
               "Initializing context for feature %s",
               clips_feature_name_.c_str());

  envs_[env_name] = clips;
  pddl_parsers_[env_name] =
      std::make_unique<clips_pddl_parser::ClipsPddlParser>(
          envs_[env_name].get_obj().get(),
          *(envs_[env_name].get_mutex_instance()), false);

  RCLCPP_DEBUG(rclcpp::get_logger(clips_feature_name_), "Initialized context!");
  return true;
}

bool PddlParserFeature::clips_context_destroyed(const std::string &env_name) {

  RCLCPP_DEBUG(rclcpp::get_logger(clips_feature_name_),
               "Destroying clips context!");

  pddl_parsers_.erase(env_name);
  envs_.erase(env_name);
  return true;
}
} // namespace cx
PLUGINLIB_EXPORT_CLASS(cx::PddlParserFeature, cx::ClipsFeature)
