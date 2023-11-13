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

#include <clipsmm.h>
#include <string>

#include "cx_core/ClipsFeature.hpp"
namespace cx {

ClipsFeature::ClipsFeature() {}
// Empty Destructor as delcared virtual
ClipsFeature::~ClipsFeature() {}

// todo: remove this file and implementation as it does nothing and is not
// necessary
std::string ClipsFeature::getFeatureName() const { return clips_feature_name; }

void ClipsFeature::initialise(const std::string &feature_name) {
  (void)feature_name;
}
void ClipsFeature::initialise(
    const std::string &feature_name,
    std::map<std::string, rclcpp::Parameter> &parameter_args) {
  parameters = parameter_args;
  initialise(feature_name);
}

} // namespace cx

