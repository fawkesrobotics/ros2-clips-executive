#ifndef CX_SKILLS_EXECUTION__SKILLEXECUTIONCONTROLLER_HPP
#define CX_SKILLS_EXECUTION__SKILLEXECUTIONCONTROLLER_HPP

#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "cx_msgs/msg/skill_execution.hpp"
#include "cx_msgs/msg/skill_executioner_information.hpp"

#include "cx_skill_execution/SkillExecutionMaster.hpp"

#include "cx_utils/NodeThread.hpp"

namespace cx {
class SkillExecutionController : public rclcpp_lifecycle::LifecycleNode

{

public:
  SkillExecutionController(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state);
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);

private:
  struct SkillMasterSt {
    std::shared_ptr<cx::SkillExecutionMaster> skill_master;
    std::shared_ptr<cx::NodeThread> skill_master_exec_node;
  };
  
  std::map<std::string, SkillMasterSt> skill_master_map_;
};
} // namespace cx

#endif // !CX_SKILLS_EXECUTION__SKILLEXECUTIONCONTROLLER_HPP