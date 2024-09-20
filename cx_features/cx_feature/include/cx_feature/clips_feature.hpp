// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  clips_feature.hpp
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

#ifndef _CX_FEATURE_CLIPS_FEATURE_HPP
#define _CX_FEATURE_CLIPS_FEATURE_HPP

#include <clips_ns/clips.h>
#include <map>
#include <string>

#include "cx_utils/LockSharedPtr.hpp"

#include "cx_msgs/msg/clips_context.hpp"

namespace cx {
/// Clips Feature
/**
 * This base class provides the interface for clips features that are managed
 * by the ClipsFeatureManager.
 *
 * Inherit from this class and export your class as plugin via pluginlib.
 */
class ClipsFeature {
public:
  ClipsFeature();
  virtual ~ClipsFeature();

  using Ptr = std::shared_ptr<cx::ClipsFeature>;

  /// Called once for each feature when it is loaded.
  virtual void initialize();

  /// Called after initialize once for every managed Clips environment.
  /**
   * Register your user-defined functions here and store the references
   * of the environment for usage.
   * The clips environment should not be locked in this function as the feature
   * manager takes care of that. Only lock the Clips environment when accessing
   * it from separate threads, such as callbacks.
   *
   * \param[in] env_name Name of the Clips environment
   * \param[in] clips a pointer to a Clips environment including a mutex to lock
   * before usage in other threads.
   *
   * \return true iff the initialization succeeded
   */
  virtual bool clips_context_init(const std::string &env_name,
                                  LockSharedPtr<clips::Environment> &clips);

  /// Called once for every managed Clips environment on shutting down the
  /// environment.
  virtual bool clips_context_destroyed(const std::string &env_name) = 0;

  std::string get_feature_name() const;

protected:
  std::string clips_feature_name_;
  /// ros parameters passed through the feature manager.
  /**
   * param_name -> param_value
   */
  std::map<std::string, rclcpp::Parameter> parameters_;
  /// store env_name -> env to remember initialized contexts.
  std::map<std::string, LockSharedPtr<clips::Environment>> envs_;

private:
  /// \internal pass name and params to the feature.
  void initialize(const std::string &feature_name,
                  std::map<std::string, rclcpp::Parameter> &parameters);

} // namespace cx

#endif // !_CX_FEATURE_CLIPS_FEATURE_HPP
