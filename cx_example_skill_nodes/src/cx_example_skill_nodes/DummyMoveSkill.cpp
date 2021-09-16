#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "cx_example_skill_nodes/DummyMoveSkill.hpp"

#include "lifecycle_msgs/msg/state.hpp"

namespace cx {

using namespace std::chrono_literals;

DummyMoveSkill::DummyMoveSkill(const std::string &id,
                               const std::string &action_name,
                               const std::chrono::nanoseconds &rate)
    : SkillExecution(id, action_name, rate) {
  executions_ = 0;
  cycles_ = 0;
}
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn
DummyMoveSkill::on_activate(const rclcpp_lifecycle::State &state) {
  std::cerr << "DummyMoveSkill::on_activate" << std::endl;
  counter_ = 0;

  return SkillExecution::on_activate(state);
}

void DummyMoveSkill::perform_execution() {
  RCLCPP_INFO_STREAM(get_logger(), "Executing [" << action_name_ << "]");
  for (const auto &param : action_parameters_) {
    RCLCPP_INFO_STREAM(get_logger(), "\t[" << param << "]");
  }

  cycles_++;

  if (counter_++ > 3) {
    finish_execution(true, 1.0, "completed");
    executions_++;
  } else {
    send_feedback(counter_ * 0.0, "running");
  }
}

} // namespace cx
