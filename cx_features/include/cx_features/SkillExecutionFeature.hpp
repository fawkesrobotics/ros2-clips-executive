#ifndef CX_FEATURES__SKILLEXECUTIONFEATURE_HPP_
#define CX_FEATURES__SKILLEXECUTIONFEATURE_HPP_

#include <map>
#include <memory>
#include <string>

#include "cx_core/ClipsFeature.hpp"
#include "cx_utils/LockSharedPtr.hpp"
#include "cx_utils/NodeThread.hpp"

#include "cx_skill_execution/SkillExecutionMaster.hpp"

namespace cx {

class SkillExecutionFeature : public ClipsFeature {
public:
  SkillExecutionFeature();
  ~SkillExecutionFeature();

  void initialise(const std::string &feature_name) override;

  bool clips_context_init(const std::string &env_name,
                          LockSharedPtr<CLIPS::Environment> &clips) override;
  bool
  clips_context_destroyed(const std::string &env_name,
                          LockSharedPtr<CLIPS::Environment> &clips) override;

  std::string getFeatureName() const;

private:
  void request_skill_execution(const std::string &env_name,
                               const std::string &skill_id,
                               const std::string &action_name,
                               const std::string &action_params,
                               const std::string &mapped_action,
                               const std::string &robot_namespace);

  void cancel_skill(const std::string &env_name,
                    const std::string &robot_namespace);

  void clips_read_skills(const std::string &env_name);

private:
  std::map<std::string, LockSharedPtr<CLIPS::Environment>> envs_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<cx::NodeThread> thread_;

  struct SkillMasterSt {
    std::shared_ptr<cx::SkillExecutionMaster> skill_master;
    std::shared_ptr<cx::NodeThread> skill_master_exec_node;
  };
  std::map<std::string, SkillMasterSt> skill_master_map_;

private:
};

} // namespace cx
#endif // !CX_FEATURES__SKILLEXECUTIONFEATURE_HPP_