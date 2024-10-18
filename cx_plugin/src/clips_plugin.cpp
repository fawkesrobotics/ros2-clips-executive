// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  ClipsPlugin.cpp
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

#include "cx_plugin/clips_plugin.hpp"
namespace cx {

ClipsPlugin::ClipsPlugin() {}
ClipsPlugin::~ClipsPlugin() {}

std::string ClipsPlugin::get_plugin_name() const { return plugin_name_; }

void ClipsPlugin::initialize() {}

void ClipsPlugin::finalize() {}

void ClipsPlugin::initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
    const std::string &plugin_name) {
  parent_ = parent;
  plugin_name_ = plugin_name;
  initialize();
}

} // namespace cx
