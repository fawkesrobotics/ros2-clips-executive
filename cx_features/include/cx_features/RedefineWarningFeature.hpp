#ifndef CX_FEATURES__REDEFINEWARNINGFEATURE_HPP_
#define CX_FEATURES__REDEFINEWARNINGFEATURE_HPP_

#include <clipsmm.h>
#include <string>

#include "cx_core/ClipsFeature.hpp"
#include "cx_utils/LockSharedPtr.hpp"

namespace cx {

class RedefineWarningFeature : public ClipsFeature {
public:
  RedefineWarningFeature();
  ~RedefineWarningFeature();

  void initialise(const std::string &feature_name) override;

  bool clips_context_init(const std::string &env_name,
                          LockSharedPtr<CLIPS::Environment> &clips) override;
  bool
  clips_context_destroyed(const std::string &env_name,
                          LockSharedPtr<CLIPS::Environment> &clips) override;

  std::string getFeatureName() const;

private:
};

} // namespace cx

#endif // !CX_FEATURES__REDEFINEWARNINGFEATURE_HPP_