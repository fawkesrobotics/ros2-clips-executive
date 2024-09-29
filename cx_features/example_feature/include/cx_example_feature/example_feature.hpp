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

  bool clips_context_init(const std::string &env_name,
                          LockSharedPtr<clips::Environment> &clips) override;
  bool clips_context_destroyed(const std::string &env_name) override;

private:
};
} // namespace cx

#endif // !CX_FEATURES__MOCKFEATURE_HPP_
