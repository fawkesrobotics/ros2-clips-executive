// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#ifndef CX_FEATURES__MOCKFEATURE_HPP_
#define CX_FEATURES__MOCKFEATURE_HPP_

#include <string>

#include "cx_core/ClipsFeature.hpp"
#include "cx_utils/LockSharedPtr.hpp"

namespace cx {

class MockFeature : public ClipsFeature {
public:
  MockFeature();
  ~MockFeature();

  void initialise(const std::string &feature_name) override;

  bool clips_context_init(const std::string &env_name,
                          LockSharedPtr<clips::Environment> &clips) override;
  bool clips_context_destroyed(const std::string &env_name) override;

  std::string getFeatureName() const;

private:
  // const std::string clips_feature_name;
};
} // namespace cx

#endif // !CX_FEATURES__MOCKFEATURE_HPP_
