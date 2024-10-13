// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#ifndef CX_FEATURES__PLANSYS2FEATURE_HPP_
#define CX_FEATURES__PLANSYS2FEATURE_HPP_

#include <string>

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "cx_feature/clips_feature.hpp"
#include "cx_utils/LockSharedPtr.hpp"

namespace cx {

class Plansys2Feature : public ClipsFeature {
public:
  Plansys2Feature();
  ~Plansys2Feature();

  bool clips_env_init(LockSharedPtr<clips::Environment> &env) override;
  bool clips_env_destroyed(LockSharedPtr<clips::Environment> &env) override;

private:
  // PLANSYS2 CLIENTS
  std::unordered_map<std::string, std::unique_ptr<plansys2::DomainExpertClient>>
      domain_client_;
  std::unordered_map<std::string,
                     std::unique_ptr<plansys2::ProblemExpertClient>>
      problem_client_;
  std::unordered_map<std::string, std::unique_ptr<plansys2::PlannerClient>>
      planner_client_;

  unsigned int plan_id_{0};

private:
  inline unsigned int next_plan_id() { return ++plan_id_; }

  void add_problem_instance(clips::Environment *env, const std::string &name,
                            const std::string &type);
  void add_problem_predicate(clips::Environment *env, const std::string &pred);
  void clear_problem_expert_knowledge(clips::Environment *env);

  void plan_with_plansys2(clips::Environment *env, const std::string &goal_id,
                          const std::string &goal, const std::string &plan_id);

  void call_planner(clips::Environment *env, const std::string &goal_id,
                    const std::string &goal, const std::string &plan_id);
};

} // namespace cx
#endif // !CX_FEATURES__PLANSYS2FEATURE_HPP_
