// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  Plansys2Feature.cpp
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

#include <string>

#include "cx_plansys2_feature/Plansys2Feature.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

using namespace std::placeholders;
namespace cx {

Plansys2Feature::Plansys2Feature() {}

Plansys2Feature::~Plansys2Feature() {}

std::string Plansys2Feature::getFeatureName() const {
  return clips_feature_name;
}

void Plansys2Feature::initialise(const std::string &feature_name) {
  clips_feature_name = feature_name;
}

bool Plansys2Feature::clips_context_init(
    const std::string &env_name, LockSharedPtr<clips::Environment> &clips) {
  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Initialising context for feature %s",
              clips_feature_name.c_str());

  envs_[env_name] = clips;
  domain_client_ = std::make_shared<plansys2::DomainExpertClient>();
  problem_client_ = std::make_shared<plansys2::ProblemExpertClient>();
  planner_client_ = std::make_shared<plansys2::PlannerClient>();

  clips::AddUDF(
      clips.get_obj().get(), "psys2-add-domain-instance", "v", 2, 2, ";sy;sy",
      [](clips::Environment * /*env*/, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        auto *instance = static_cast<Plansys2Feature *>(udfc->context);
        clips::UDFValue name, type;
        using namespace clips;
        clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &name);
        clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &type);

        instance->addProblemInstance(name.lexemeValue->contents,
                                     type.lexemeValue->contents);
      },
      "addProblemInstance", this);

  clips::AddUDF(
      clips.get_obj().get(), "psys2-add-domain-predicate", "v", 1, 1, ";sy",
      [](clips::Environment * /*env*/, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        auto *instance = static_cast<Plansys2Feature *>(udfc->context);
        clips::UDFValue predicate;
        using namespace clips;
        clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &predicate);
        instance->addProblemPredicate(predicate.lexemeValue->contents);
      },
      "addProblemPredicate", this);

  clips::AddUDF(
      clips.get_obj().get(), "psys2-clear-knowledge", "v", 0, 0, "",
      [](clips::Environment * /*env*/, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        auto *instance = static_cast<Plansys2Feature *>(udfc->context);
        instance->clearProblemExpertKnowledge();
      },
      "clearProblemExpertKnowledge", this);

  clips::AddUDF(
      clips.get_obj().get(), "psys2-call-planner", "v", 3, 3, ";sy;sy;sy",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        auto *instance = static_cast<Plansys2Feature *>(udfc->context);
        clips::UDFValue plan;
        clips::UDFValue domain;
        clips::UDFValue problem;
        using namespace clips;
        clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &plan);
        clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &domain);
        clips::UDFNthArgument(udfc, 3, LEXEME_BITS, &problem);
        instance->planWithPlansys2(env, plan.lexemeValue->contents,
                                   domain.lexemeValue->contents,
                                   problem.lexemeValue->contents);
      },
      "planWithPlansys2", this);

  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name), "Initialised context!");
  return true;
}

bool Plansys2Feature::clips_context_destroyed(const std::string &env_name) {

  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Destroying clips context!");
  clips::RemoveUDF(envs_[env_name].get_obj().get(),
                   "psys2-add-domain-instance");
  clips::RemoveUDF(envs_[env_name].get_obj().get(),
                   "psys2-add-domain-predicate");
  clips::RemoveUDF(envs_[env_name].get_obj().get(), "psys2-clear-knowledge");
  clips::RemoveUDF(envs_[env_name].get_obj().get(), "psys2-call-planner");
  envs_.erase(env_name);

  return true;
}

void Plansys2Feature::addProblemInstance(const std::string &name,
                                         const std::string &type) {
  const std::string log_name = "PSYS2CLIPS";

  RCLCPP_INFO(rclcpp::get_logger(log_name),
              "Trying to add instance: %s - %s to problem expert", name.c_str(),
              type.c_str());
  bool success = problem_client_->addInstance(plansys2::Instance(name, type));
  if (success) {
    RCLCPP_INFO(rclcpp::get_logger(log_name),
                "Successfully added instance: %s - %s to problem expert",
                name.c_str(), type.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(log_name),
                 "Problem adding instance: %s - %s to problem expert",
                 name.c_str(), type.c_str());
  }
}

void Plansys2Feature::addProblemPredicate(const std::string &pred) {
  const std::string log_name = "PSYS2CLIPS";
  RCLCPP_INFO(rclcpp::get_logger(log_name),
              "Trying to add predicate: %s to problem expert", pred.c_str());
  bool success = problem_client_->addPredicate(plansys2::Predicate(pred));
  if (success) {
    RCLCPP_INFO(rclcpp::get_logger(log_name),
                "Successfully added predicate: %s to problem expert",
                pred.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(log_name),
                 "Problem adding predicate: %s to problem expert",
                 pred.c_str());
  }
}

void Plansys2Feature::clearProblemExpertKnowledge() {
  const std::string log_name = "PSYS2CLIPS";
  RCLCPP_INFO(rclcpp::get_logger(log_name),
              "Clearing problem expert knowledge!");
  problem_client_->clearKnowledge();
}

void Plansys2Feature::planWithPlansys2(clips::Environment *env,
                                       const std::string &goal_id,
                                       const std::string &goal,
                                       const std::string &plan_id) {
  const std::string log_name = "PSYS2CLIPS";
  RCLCPP_INFO(rclcpp::get_logger(log_name), "planWithPlansys2 called! id: %s",
              plan_id.c_str());

  call_planner(env, goal_id, goal, plan_id);
}

void Plansys2Feature::call_planner(clips::Environment *env,
                                   const std::string &goal_id,
                                   const std::string &goal,
                                   const std::string &plan_id) {
  const std::string log_name = "PSYS2CLIPS";

  if (!problem_client_->setGoal(plansys2::Goal(goal))) {
    RCLCPP_ERROR(rclcpp::get_logger(log_name),
                 "Couldn't set goal '%s' with id %s for problem expert",
                 goal.c_str(), goal_id.c_str());
    return;
  }
  auto domain = domain_client_->getDomain();
  auto problem = problem_client_->getProblem();

  RCLCPP_ERROR(rclcpp::get_logger(log_name),
               "Calling plan for goal '%s' with id %s", goal.c_str(),
               goal_id.c_str());
  auto plan = planner_client_->getPlan(domain, problem);
  // std::lock_guard<std::mutex> guard(*(clips.get_mutex_instance()));

  if (!plan.has_value()) {
    RCLCPP_ERROR(rclcpp::get_logger(log_name),
                 "Could not find plan to reach goal '%s' with id %s",
                 goal.c_str(), goal_id.c_str());
    RCLCPP_ERROR(rclcpp::get_logger(log_name),
                 "(pddl-plan-feedback (status PLAN-FAILED) (plan-id %s))",
                 plan_id.c_str());
    clips::AssertString(
        env,
        std::format("(pddl-plan-feedback (status PLAN-FAILED) (plan-id {}))",
                    plan_id)
            .c_str());
    return;
  }

  const std::string space_delimiter = " ";
  unsigned int id = 1;

  clips::AssertString(
      env, std::format("(plan (id {}) (goal-id {}) (type SEQUENTIAL))", plan_id,
                       goal_id)
               .c_str());
  for (const auto &plan_item : plan.value().items) {
    if (plan_item.action != "") {
      const std::string &action = plan_item.action;
      auto pos = action.find(space_delimiter);
      const std::string action_name = action.substr(1, pos - 1);
      const std::string param_values =
          action.substr(pos + space_delimiter.length(), action.length() - 2);

      RCLCPP_INFO(rclcpp::get_logger(log_name),
                  "(plan-action (id %d) (goal-id %s) (plan-id %s) "
                  "(action-name %s) (param-values %s))",
                  id, goal_id.c_str(), plan_id.c_str(), action_name.c_str(),
                  param_values.c_str());

      clips::AssertString(
          env, std::format("(plan-action (id {}) (goal-id {}) (plan-id {}) "
                           "(action-name {}) (param-values {}))",
                           id, goal_id, plan_id, action_name, param_values)
                   .c_str());
      id++;
    }
  }
  clips::AssertString(
      env,
      std::format("(pddl-plan-feedback (status PLANNED) (plan-id {}))", plan_id)
          .c_str());
}

} // namespace cx

PLUGINLIB_EXPORT_CLASS(cx::Plansys2Feature, cx::ClipsFeature)
