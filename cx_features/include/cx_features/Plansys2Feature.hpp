#ifndef CX_FEATURES__PLANSYS2FEATURE_HPP_
#define CX_FEATURES__PLANSYS2FEATURE_HPP_

#include <string>

#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "cx_core/ClipsFeature.hpp"
#include "cx_utils/LockSharedPtr.hpp"

namespace cx {

class Plansys2Feature : public ClipsFeature {
public:
  Plansys2Feature();
  ~Plansys2Feature();

  void initialise(const std::string &feature_name) override;

  bool clips_context_init(const std::string &env_name,
                          LockSharedPtr<CLIPS::Environment> &clips) override;
  bool
  clips_context_destroyed(const std::string &env_name,
                          LockSharedPtr<CLIPS::Environment> &clips) override;

  std::string getFeatureName() const;

private:
  std::map<std::string, LockSharedPtr<CLIPS::Environment>> envs_;
};

} // namespace cx
#endif // !CX_FEATURES__PLANSYS2FEATURE_HPP_