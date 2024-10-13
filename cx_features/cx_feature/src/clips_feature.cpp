// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  ClipsFeature.cpp
 *
 *  Created: 22 June 2021
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

#include <string>

#include "cx_feature/clips_feature.hpp"
namespace cx {

ClipsFeature::ClipsFeature() {}
ClipsFeature::~ClipsFeature() {}

std::string ClipsFeature::get_feature_name() const { return feature_name_; }

void ClipsFeature::initialize() {}

void ClipsFeature::initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
    const std::string &feature_name) {
  parent_ = parent;
  feature_name_ = feature_name;
  initialize();
}

} // namespace cx
