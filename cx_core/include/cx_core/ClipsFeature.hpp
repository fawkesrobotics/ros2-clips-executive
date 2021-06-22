#ifndef _CX_CORE_CLIPS_FEATURE_HPP
#define _CX_CORE_CLIPS_FEATURE_HPP

#include <clipsmm.h>
#include <string>

#include "cx_utils/LockSharedPtr.hpp"

namespace cx {
class ClipsFeature {
public:
  ClipsFeature(const std::string &feature_name);
  virtual ~ClipsFeature();
  // TODO DESCRIPTION
  // Initialises a CLIPS context to use the provided feature.
  virtual void clips_context_init(const std::string &env_name,
                                  LockSharedPtr<CLIPS::Environment> &clips) = 0;
  virtual void clips_context_destroyed(const std::string &env_name) = 0;

protected:
  const std::string clips_feature_name;
};

} // namespace cx

#endif // !_CX_CORE_CLIPS_FEATURE_HPP