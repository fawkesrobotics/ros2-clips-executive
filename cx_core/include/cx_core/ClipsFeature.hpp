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
  ClipsFeature(const std::string &feature_name);
  virtual ~ClipsFeature();
  // TODO DESCRIPTION
  // Initialises a CLIPS context to use the provided feature.
  virtual bool clips_context_init(const std::string &env_name,
                                  LockSharedPtr<CLIPS::Environment> &clips) = 0;
  virtual bool
  clips_context_destroyed(const std::string &env_name,
                          LockSharedPtr<CLIPS::Environment> &clips) = 0;
  std::string getFeatureName() const;

protected:
  const std::string clips_feature_name;
  // rclcpp::Node::SharedPtr node_;
};

} // namespace cx

#endif // !_CX_CORE_CLIPS_FEATURE_HPP