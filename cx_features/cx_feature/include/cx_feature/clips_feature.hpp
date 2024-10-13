// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#ifndef _CX_FEATURE_CLIPS_FEATURE_HPP
#define _CX_FEATURE_CLIPS_FEATURE_HPP

#include <clips_ns/clips.h>
#include <map>
#include <string>

#include "cx_utils/LockSharedPtr.hpp"

#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace cx {

/// Clips Feature
/**
 * This base class provides the interface for clips features that are managed
 * by the ClipsFeatureManager.
 *
 * Inherit from this class and export your class as plugin via pluginlib.
 */
class ClipsFeature {
  friend class ClipsFeatureManager;
  using Ptr = pluginlib::UniquePtr<cx::ClipsFeature>;

public:
  ClipsFeature();
  virtual ~ClipsFeature();

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
  virtual bool clips_env_init(LockSharedPtr<clips::Environment> &clips) = 0;

  /// Called once for every managed Clips environment on shutting down the
  /// environment.
  virtual bool
  clips_env_destroyed(LockSharedPtr<clips::Environment> &clips) = 0;

  std::string get_feature_name() const;

protected:
  std::string feature_name_;

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;

private:
  /// \internal pass name and params to the feature.
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
                  const std::string &feature_name);
};

} // namespace cx

#endif // !_CX_FEATURE_CLIPS_FEATURE_HPP
