// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  ClipsFeature.hpp
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

#ifndef _CX_CORE_CLIPS_FEATURE_HPP
#define _CX_CORE_CLIPS_FEATURE_HPP

#include <clipsmm.h>
#include <map>
#include <string>

#include "cx_utils/LockSharedPtr.hpp"

#include "cx_msgs/msg/clips_context.hpp"

namespace cx {
class ClipsFeature {
public:
  ClipsFeature();
  virtual ~ClipsFeature();

  using Ptr = std::shared_ptr<cx::ClipsFeature>;

  virtual void initialise(const std::string &feature_name);
  virtual void initialise(const std::string &feature_name,
                          std::map<std::string, rclcpp::Parameter> &parameters);
  // Provides feature functionality to CLIPS.
  virtual bool clips_context_init(const std::string &env_name,
                                  LockSharedPtr<CLIPS::Environment> &clips) = 0;
  virtual bool clips_context_destroyed(const std::string &env_name) = 0;
  std::string getFeatureName() const;

protected:
  std::string clips_feature_name;
  std::map<std::string, rclcpp::Parameter> parameters;
};

} // namespace cx

#endif // !_CX_CORE_CLIPS_FEATURE_HPP
