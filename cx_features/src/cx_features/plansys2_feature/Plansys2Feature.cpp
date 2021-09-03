#include <string>

#include "cx_features/Plansys2Feature.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

using std::placeholders::_1;
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
    const std::string &env_name, LockSharedPtr<CLIPS::Environment> &clips) {
  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Initialising context for feature %s",
              clips_feature_name.c_str());

  envs_[env_name] = clips;
  domain_client_ = std::make_shared<plansys2::DomainExpertClient>();
  problem_client_ = std::make_shared<plansys2::ProblemExpertClient>();
  planner_client_ = std::make_shared<plansys2::PlannerClient>();

  clips->add_function(
      "psys2-add-domain-instance",
      sigc::slot<void, std::string, std::string>(sigc::bind<0>(
          sigc::mem_fun(*this, &Plansys2Feature::addProblemInstance),
          env_name)));
  clips->add_function(
      "psys2-add-domain-predicate",
      sigc::slot<void, std::string>(sigc::bind<0>(
          sigc::mem_fun(*this, &Plansys2Feature::addProblemPredicate),
          env_name)));
  clips->add_function(
      "psys2-clear-knowledge",
      sigc::slot<void>(sigc::bind<0>(
          sigc::mem_fun(*this, &Plansys2Feature::clearProblemExpertKnowledge),
          env_name)));
  clips->add_function(
      "psys2-call-planner",
      sigc::slot<bool, std::string, std::string>(sigc::bind<0>(
          sigc::mem_fun(*this, &Plansys2Feature::planWithPlansys2), env_name)));

  // clips->evaluate(
  //     "(psys2-call-planner \"TESTGOAL\" \"(and (ivo at coffemachine))\")");
  // clips->evaluate("(psys2-add-domain-predicate \"(ivo at computer)\")");
  // clips->evaluate("(psys2-add-domain-instance \"ivo\" \"robot\")");
  // clips->evaluate("(psys2-clear-knowledge)");

  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name), "Initialised context!");
  return true;
}

bool Plansys2Feature::clips_context_destroyed(
    const std::string &env_name, LockSharedPtr<CLIPS::Environment> &clips) {

  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Destroying clips context!");
  envs_.erase(env_name);

  return true;
}

void Plansys2Feature::addProblemInstance(const std::string &env_name,
                                         const std::string &name,
                                         const std::string &type) {
  const std::string log_name = std::move("PSYS2CLIPS|" + env_name);

  if (envs_.find(env_name) == envs_.end()) {
    RCLCPP_ERROR(rclcpp::get_logger(log_name),
                 "Environment %s has not been registered "
                 "for plansys2 feature -> can't add instance",
                 env_name.c_str());
    return;
  }
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

void Plansys2Feature::addProblemPredicate(const std::string &env_name,
                                          const std::string &pred) {
  const std::string log_name = std::move("PSYS2CLIPS|" + env_name);

  if (envs_.find(env_name) == envs_.end()) {
    RCLCPP_ERROR(rclcpp::get_logger(log_name),
                 "Environment %s has not been registered "
                 "for plansys2 feature -> can't add predicate",
                 env_name.c_str());
    return;
  }
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

void Plansys2Feature::clearProblemExpertKnowledge(const std::string &env_name) {
  const std::string log_name = std::move("PSYS2CLIPS|" + env_name);
  if (envs_.find(env_name) == envs_.end()) {
    RCLCPP_ERROR(rclcpp::get_logger(log_name),
                 "Environment %s has not been registered "
                 "for plansys2 feature -> can't clear knowledge",
                 env_name.c_str());
    return;
  }
  problem_client_->clearKnowledge();
}

bool Plansys2Feature::planWithPlansys2(const std::string &env_name,
                                       const std::string &goal_id,
                                       const std::string &goal) {
  const std::string log_name = std::move("PSYS2CLIPS|" + env_name);
  if (envs_.find(env_name) == envs_.end()) {
    RCLCPP_ERROR(rclcpp::get_logger(log_name),
                 "Environment %s has not been registered "
                 "for plansys2 feature -> can't plan for goal '%s' with id %s",
                 env_name.c_str(), goal.c_str(), goal_id.c_str());
    return false;
  }

  if (!problem_client_->setGoal(plansys2::Goal(goal))) {
    RCLCPP_ERROR(rclcpp::get_logger(log_name),
                 "Couldn't set goal '%s' with id %s for problem expert",
                 goal.c_str(), goal_id.c_str());
    return false;
  }

  auto domain = domain_client_->getDomain();
  auto problem = problem_client_->getProblem();
  RCLCPP_ERROR(rclcpp::get_logger(log_name),
               "Calling plan for goal '%s' with id %s", goal.c_str(),
               goal_id.c_str());
  auto plan = planner_client_->getPlan(domain, problem);

  if (!plan.has_value()) {
    RCLCPP_ERROR(rclcpp::get_logger(log_name),
                 "Could not find plan to reach goal '%s' with id %s",
                 goal.c_str(), goal_id.c_str());
    return false;
  }
  // const auto &plan_items = plan.value();
  cx::LockSharedPtr<CLIPS::Environment> clips = envs_[env_name];

  int id = 1;
  int plan_id = next_plan_id();
  RCLCPP_INFO(rclcpp::get_logger(log_name), "Next plan id: %d", plan_id);
  const std::string space_delimiter = " ";
  clips.scopedLock();

  clips->assert_fact_f("(plan (id %s) (goal-id %s))",
                       std::to_string(plan_id).c_str(), goal_id.c_str());
  for (const auto &plan_item : plan.value().items) {
    if (plan_item.action != "") {
      const std::string &action = plan_item.action;
      auto pos = action.find(space_delimiter);
      const std::string action_name = action.substr(1, pos);
      const std::string param_values =
          action.substr(pos + space_delimiter.length(), action.length() - 2);

      RCLCPP_INFO(rclcpp::get_logger(log_name),
                  "Action naem '%s' with param-values %s", action_name.c_str(),
                  param_values.c_str());
      clips->assert_fact_f("(plan-action (id %s) (goal-id %s) (plan-id %s) "
                           "(action-name %s) (param-values %s))",
                           std::to_string(id).c_str(), goal_id.c_str(),
                           std::to_string(plan_id).c_str(), action_name.c_str(),
                           param_values.c_str());
      id++;
    }
  }
  return true;
}

} // namespace cx

PLUGINLIB_EXPORT_CLASS(cx::Plansys2Feature, cx::ClipsFeature)
