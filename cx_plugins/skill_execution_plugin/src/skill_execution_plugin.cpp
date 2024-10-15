// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  SkillExecutionPlugin.cpp
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

#include <format>
#include <string>

#include "cx_skill_execution/SkillExecutionMaster.hpp"
#include "cx_skill_execution_plugin/skill_execution_plugin.hpp"

#include <cx_utils/clips_env_context.hpp>

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

using namespace std::placeholders;
namespace cx {

using namespace std::chrono_literals;

SkillExecutionPlugin::SkillExecutionPlugin() {}

SkillExecutionPlugin::~SkillExecutionPlugin() {}

bool SkillExecutionPlugin::clips_env_init(
    LockSharedPtr<clips::Environment> &env) {
  RCLCPP_INFO(rclcpp::get_logger(plugin_name_),
              "Initializing context for plugin %s", plugin_name_.c_str());

  node_ = rclcpp::Node::make_shared("skill_execution_plugin");
  // spin node
  thread_ = std::make_unique<cx::NodeThread>(node_->get_node_base_interface());

  clips::AddUDF(
      env.get_obj().get(), "call-skill-execution", "v", 6, 6,
      ";sy;sy;sy;sy;sy;sy",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        SkillExecutionPlugin *instance =
            static_cast<SkillExecutionPlugin *>(udfc->context);

        clips::UDFValue skill_id, action_name, action_params, mapped_action,
            robot_id, executor_id;
        using namespace clips;
        clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &skill_id);
        clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &action_name);
        clips::UDFNthArgument(udfc, 3, LEXEME_BITS, &action_params);
        clips::UDFNthArgument(udfc, 4, LEXEME_BITS, &mapped_action);
        clips::UDFNthArgument(udfc, 5, LEXEME_BITS, &robot_id);
        clips::UDFNthArgument(udfc, 6, LEXEME_BITS, &executor_id);

        instance->request_skill_execution(env, skill_id.lexemeValue->contents,
                                          action_name.lexemeValue->contents,
                                          action_params.lexemeValue->contents,
                                          mapped_action.lexemeValue->contents,
                                          robot_id.lexemeValue->contents,
                                          executor_id.lexemeValue->contents);
      },
      "request_skill_execution", this);

  clips::AddUDF(
      env.get_obj().get(), "call-skill-cancel", "v", 2, 2, ";sy;sy",
      [](clips::Environment * /*env*/, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        SkillExecutionPlugin *instance =
            static_cast<SkillExecutionPlugin *>(udfc->context);

        clips::UDFValue robot_id, executor_id;

        using namespace clips;
        clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &robot_id);
        clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &executor_id);

        instance->cancel_skill(robot_id.lexemeValue->contents,
                               executor_id.lexemeValue->contents);
      },
      "cancel_skill", this);

  RCLCPP_INFO(rclcpp::get_logger(plugin_name_), "Initialized context!");
  return true;
}

bool SkillExecutionPlugin::clips_env_destroyed(
    LockSharedPtr<clips::Environment> &env) {

  RCLCPP_INFO(rclcpp::get_logger(plugin_name_), "Destroying clips context!");
  clips::RemoveUDF(env.get_obj().get(), "call-skill-execution");
  clips::RemoveUDF(env.get_obj().get(), "call-skill-cancel");
  return true;
}

void SkillExecutionPlugin::request_skill_execution(
    clips::Environment *env, const std::string &skill_id,
    const std::string &action_name, const std::string &action_params,
    const std::string &mapped_action, const std::string &robot_id,
    const std::string &executor_id) {

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
      clips::AssertString(
          env,
          std::format("(skill-feedback (skill-id (sym-cat \"{}\")) (robot "
                      "\"{}\") (executor \"{}\") (status S_FAILED) (error "
                      "\"other skill already running\") (time (now)))",
                      skill_id.c_str(), robot_id.c_str(), executor_id.c_str())
              .c_str());

      return;
      // Abort skill execution for the same agent id
    }
  }
  auto context = CLIPSEnvContext::get_context(env);
  RCLCPP_INFO(node_->get_logger(),
              "Requesting skill (%s, %s) with mapped_action %s for robot %s "
              "and executor %s for %s ",
              action_name.c_str(), action_params.c_str(), mapped_action.c_str(),
              robot_id.c_str(), executor_id.c_str(), skill_id.c_str());
  // make new skill execution master
  auto skill_master = std::make_shared<cx::SkillExecutionMaster>(
      /*name the master as the provided skill id*/ skill_id, skill_id,
      action_name, mapped_action, action_params, robot_id, executor_id,
      context->env_lock_ptr_);

  auto skill_master_st = SkillMasterSt();
  skill_master_st.skill_master = skill_master;

  // Spin exec node in executor
  skill_master_st.skill_master_exec_node =
      std::make_shared<cx::NodeThread>(skill_master->get_node_base_interface());

  skill_master->request_skill_execution();
  skill_master_map_[id] = skill_master_st;
}

void SkillExecutionPlugin::cancel_skill(const std::string &robot_id,
                                        const std::string &executor_id) {
  std::pair<std::string, std::string> id{robot_id, executor_id};
  if (skill_master_map_.find(id) != skill_master_map_.end()) {
    skill_master_map_[id].skill_master->cancel_execution();
    // cx::LockSharedPtr<clips::Environment> &clips = envs_[env_name];
    //  std::lock_guard<std::mutex> guard(*(clips.get_mutex_instance()));
    //   clips->assert_fact_f();
  } else {
    RCLCPP_ERROR(node_->get_logger(),
                 "Requested skill can't be cancelled, as it is not Present!");
  }
}
} // namespace cx

PLUGINLIB_EXPORT_CLASS(cx::SkillExecutionPlugin, cx::ClipsPlugin)
