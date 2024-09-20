// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

/***************************************************************************
 *  SkillExecutionFeature.hpp
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

#ifndef CX_FEATURES__SKILLEXECUTIONFEATURE_HPP_
#define CX_FEATURES__SKILLEXECUTIONFEATURE_HPP_

#include <map>
#include <memory>
#include <string>

#include "cx_feature/clips_feature.hpp"
#include "cx_utils/LockSharedPtr.hpp"
#include "cx_utils/NodeThread.hpp"

#include "cx_skill_execution/SkillExecutionMaster.hpp"

namespace cx {

class SkillExecutionFeature : public ClipsFeature {
public:
  SkillExecutionFeature();
  ~SkillExecutionFeature();

  void initialize(const std::string &feature_name) override;

  bool clips_context_init(const std::string &env_name,
                          LockSharedPtr<clips::Environment> &clips) override;
  bool clips_context_destroyed(const std::string &env_name) override;

  std::string getFeatureName() const;

private:
  void request_skill_execution(clips::Environment *env,
                               const std::string &skill_id,
                               const std::string &action_name,
                               const std::string &action_params,
                               const std::string &mapped_action,
                               const std::string &robot_id,
                               const std::string &executor_id);

  void cancel_skill(const std::string &robot_id,
                    const std::string &executor_id);

  void clips_read_skills(const std::string &env_name);

private:
  std::map<std::string, LockSharedPtr<clips::Environment>> envs_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<cx::NodeThread> thread_;

  struct SkillMasterSt {
    std::shared_ptr<cx::SkillExecutionMaster> skill_master;
    std::shared_ptr<cx::NodeThread> skill_master_exec_node;
  };
  std::map<std::pair<std::string, std::string>, SkillMasterSt>
      skill_master_map_;

private:
};

} // namespace cx
#endif // !CX_FEATURES__SKILLEXECUTIONFEATURE_HPP_
