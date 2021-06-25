#ifndef CX_FEATURES__MOCKFEATURE_HPP_
#define CX_FEATURES__MOCKFEATURE_HPP_

#include <string>

// #include "cx_clips/CLIPSEnvManagerClient.hpp"
// #include "cx_clips/CLIPSEnvManagerNode.h"

#include "cx_core/ClipsFeature.hpp"
#include "cx_msgs/msg/clips_context.hpp"
#include "cx_utils/LockSharedPtr.hpp"

namespace cx {

class MockFeature : public ClipsFeature {
public:
  explicit MockFeature(const std::string &feature_name);
  ~MockFeature();
  bool clips_context_init(const std::string &env_name,
                          LockSharedPtr<CLIPS::Environment> &clips);
  bool clips_context_destroyed(const std::string &env_name,
                               LockSharedPtr<CLIPS::Environment> &clips);
  // void addEnvironment(const std::string &env_name,
  //                     LockSharedPtr<CLIPS::Environment> &clips);

  std::string getFeatureName() const;

private:
  // const std::string clips_feature_name;
};
} // namespace cx

#endif // !CX_FEATURES__MOCKFEATURE_HPP_
