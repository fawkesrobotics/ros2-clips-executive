/***************************************************************************
 *  SkillExecutionFeature.cpp
 *
 *  Created: 16 September 2021
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

#include "cx_features/SkillExecutionFeature.hpp"
#include "cx_skill_execution/SkillExecutionMaster.hpp"

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

using namespace std::placeholders;
namespace cx {

using namespace std::chrono_literals;

SkillExecutionFeature::SkillExecutionFeature() {}

SkillExecutionFeature::~SkillExecutionFeature() {}

std::string SkillExecutionFeature::getFeatureName() const {
  return clips_feature_name;
}

void SkillExecutionFeature::initialise(const std::string &feature_name) {
  clips_feature_name = feature_name;
}

bool SkillExecutionFeature::clips_context_init(
    const std::string &env_name, LockSharedPtr<CLIPS::Environment> &clips) {
  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Initialising context for feature %s",
              clips_feature_name.c_str());

  envs_[env_name] = clips;
  node_ = rclcpp::Node::make_shared("skill_execution_feature");
  // spin node
  thread_ = std::make_unique<cx::NodeThread>(node_->get_node_base_interface());

  clips->add_function(
      "call-skill-execution",
      sigc::slot<void, std::string, std::string, std::string, std::string,
                 std::string, std::string>(sigc::bind<0>(
          sigc::mem_fun(*this, &SkillExecutionFeature::request_skill_execution),
          env_name)));
  clips->add_function(
      "call-skill-cancel",
      sigc::slot<void, std::string, std::string>(sigc::bind<0>(
          sigc::mem_fun(*this, &SkillExecutionFeature::cancel_skill),
          env_name)));

  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name), "Initialised context!");
  return true;
}

bool SkillExecutionFeature::clips_context_destroyed(
    const std::string &env_name) {

  RCLCPP_INFO(rclcpp::get_logger(clips_feature_name),
              "Destroying clips context!");
  envs_.erase(env_name);

  return true;
}

void SkillExecutionFeature::request_skill_execution(
    const std::string &env_name, const std::string &skill_id,
    const std::string &action_name, const std::string &action_params,
    const std::string &mapped_action, const std::string &robot_id,
    const std::string &executor_id) {

  if (envs_.find(env_name) == envs_.end()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Environment %s has not been registered "
                 "for skill_execution feature -> can't request skill",
                 env_name.c_str());
    return;
  }
  cx::LockSharedPtr<CLIPS::Environment> clips = envs_[env_name];
  std::pair<std::string, std::string> id{robot_id, executor_id};

  // If there is currently running skill, which is unlikely to happen
  auto old_entry = skill_master_map_.find(id);
  if (old_entry != skill_master_map_.end()) {
    auto exec_info = skill_master_map_[id].skill_master->get_exec_info();

    if (exec_info.status == cx_msgs::msg::SkillActionExecInfo::S_FINAL ||
        exec_info.status == cx_msgs::msg::SkillActionExecInfo::S_FAILED) {
      skill_master_map_.erase(id);
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Previous skill running, abort!");
      clips->assert_fact_f("(skill-feedback (skill-id (sym-cat \"%s\")) (robot "
                           "\"%s\") (executor \"%s\") (status S_FAILED) (error "
                           "\"other skill already running\") (time (now)))",
                           skill_id.c_str(), robot_id.c_str(),
                           executor_id.c_str());

      return;
      // Abort skill execution for the same agent id
    }
  }
  RCLCPP_INFO(node_->get_logger(),
              "Requesting skill (%s, %s) with mapped_action %s for robot %s "
              "and executor %s for %s ",
              action_name.c_str(), action_params.c_str(), mapped_action.c_str(),
              robot_id.c_str(), executor_id.c_str(), skill_id.c_str());
  // make new skill execution master
  auto skill_master = std::make_shared<cx::SkillExecutionMaster>(
      /*name the master as the provided skill id*/ skill_id, skill_id,
      action_name, mapped_action, action_params, robot_id, executor_id,
      envs_[env_name]);

  auto skill_master_st = SkillMasterSt();
  skill_master_st.skill_master = skill_master;

  // Spin exec node in executor
  skill_master_st.skill_master_exec_node =
      std::make_shared<cx::NodeThread>(skill_master->get_node_base_interface());

  skill_master->request_skill_execution();
  skill_master_map_[id] = skill_master_st;
}

void SkillExecutionFeature::cancel_skill(const std::string &env_name,
                                         const std::string &robot_id,
                                         const std::string &executor_id) {
  if (envs_.find(env_name) == envs_.end()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Environment %s has not been registered "
                 "for skill_execution feature -> can't request skill",
                 env_name.c_str());
    return;
  }
  std::pair<std::string, std::string> id{robot_id, executor_id};
  if (skill_master_map_.find(id) != skill_master_map_.end()) {
    skill_master_map_[id].skill_master->cancel_execution();
    // cx::LockSharedPtr<CLIPS::Environment> &clips = envs_[env_name];
    //  std::lock_guard<std::mutex> guard(*(clips.get_mutex_instance()));
    //   clips->assert_fact_f();
  } else {
    RCLCPP_ERROR(node_->get_logger(),
                 "Requested skill can't be cancelled, as it is not Present!");
  }
}
} // namespace cx

PLUGINLIB_EXPORT_CLASS(cx::SkillExecutionFeature, cx::ClipsFeature)
