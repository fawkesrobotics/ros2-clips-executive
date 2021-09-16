#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <regex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "cx_skill_execution/SkillExecution.hpp"

#include "cx_msgs/msg/skill_action_execinfo.hpp"
#include "cx_msgs/msg/skill_execution.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace cx {
using namespace std::chrono_literals;

class DummyMoveSkill : public cx::SkillExecution {

  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

public:
  // using Ptr = std::shared_ptr<DummyMoveSkill>;
  // static Ptr make_shared(const std::string &node_name,
  //                        const std::string &action,
  //                        const std::chrono::nanoseconds &rate) {
  //   return std::make_shared<DummyMoveSkill>(node_name, action, rate);
  // }

  DummyMoveSkill(const std::string &id, const std::string &action_name,
                 const std::chrono::nanoseconds &rate);

  CallbackReturn on_activate(const rclcpp_lifecycle::State &state);

  void perform_execution() override;

  int counter_;
  int executions_;
  int cycles_;
};
} // namespace cx
