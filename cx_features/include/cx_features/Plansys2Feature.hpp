/***************************************************************************
 *  Plansys2Feature.hpp
 *
 *  Created: 02 September 2021
 *  Copyright  2021  Ivaylo Doychev
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef CX_FEATURES__PLANSYS2FEATURE_HPP_
#define CX_FEATURES__PLANSYS2FEATURE_HPP_

#include <string>

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
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
  bool clips_context_destroyed(const std::string &env_name) override;

  std::string getFeatureName() const;

private:
  std::map<std::string, LockSharedPtr<CLIPS::Environment>> envs_;
  // PLANSYS2 CLIENTS
  std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;

  unsigned int plan_id_{0};

private:
  inline unsigned int next_plan_id() { return ++plan_id_; }

  void addProblemInstance(const std::string &env_name, const std::string &name,
                          const std::string &type);
  void addProblemPredicate(const std::string &env_name,
                           const std::string &pred);
  void clearProblemExpertKnowledge(const std::string &env_name);

  void planWithPlansys2(const std::string &env_name, const std::string &goal_id,
                        const std::string &goal, const std::string &plan_id);

  void call_planner(const std::string &env_name, const std::string &goal_id,
                    const std::string &goal, const std::string &plan_id);
};

} // namespace cx
#endif // !CX_FEATURES__PLANSYS2FEATURE_HPP_