// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#ifndef CX_FEATURES__MOCKFEATURE_HPP_
#define CX_FEATURES__MOCKFEATURE_HPP_

#include <string>

#include "cx_feature/clips_feature.hpp"
#include "cx_utils/LockSharedPtr.hpp"

namespace cx {

class ExampleFeature : public ClipsFeature {
public:
  ExampleFeature();
  ~ExampleFeature();

  void initialize();

  bool clips_env_init(LockSharedPtr<clips::Environment> &env) override;
  bool clips_env_destroyed(LockSharedPtr<clips::Environment> &env) override;

private:
  std::unique_ptr<rclcpp::Logger> logger_;
};
} // namespace cx

#endif // !CX_FEATURES__MOCKFEATURE_HPP_
