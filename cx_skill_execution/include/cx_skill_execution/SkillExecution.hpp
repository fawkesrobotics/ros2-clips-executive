/*
  Classes implmenting a certain action should
  inherit from the class and implement their
  individual behaviour.
*/

#ifndef CX_SKILLS_EXECUTION__SKILLEXECUTION_HPP
#define CX_SKILLS_EXECUTION__SKILLEXECUTION_HPP

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "cx_msgs/msg/skill_execution.hpp"
#include "cx_msgs/msg/skill_executioner_information.hpp"

namespace cx {

class SkillExecution : public rclcpp_lifecycle::LifecycleNode {

public:
  SkillExecution(const std::string &node_name, const std::string &action_name,
                 const std::chrono::nanoseconds &pub_rate,
                 const rclcpp::NodeOptions &options = rclcpp::NodeOptions(),
                 const std::string &namespace_ = "");

protected:
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state);
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);

  virtual void perform_execution() {};

  void skill_board_cb(const cx_msgs::msg::SkillExecution::SharedPtr msg);

  void send_response(const cx_msgs::msg::SkillExecution::SharedPtr msg);
  void send_feedback(float progress, const std::string &status);
  void finish_execution(bool success, float progress,
                        const std::string &status);

  const std::string action_name_;
  std::vector<std::string> action_parameters_;
  std::string mapped_action_;
  const std::chrono::nanoseconds exec_rate_;
  bool commited_to_skill_;

  rclcpp_lifecycle::LifecyclePublisher<cx_msgs::msg::SkillExecution>::SharedPtr
      skill_board_pub;
  rclcpp::Subscription<cx_msgs::msg::SkillExecution>::SharedPtr skill_board_sub;
  // Creates wall timer for the implemented working function
  rclcpp::TimerBase::SharedPtr execution_heartbeat_;

  rclcpp_lifecycle::LifecyclePublisher<
      cx_msgs::msg::SkillExecutionerInformation>::SharedPtr
      executioner_info_pub_;
  cx_msgs::msg::SkillExecutionerInformation executioner_info_;
  rclcpp::TimerBase::SharedPtr executioner_info_heartbeat_;
};
} // namespace cx

#endif // !CX_SKILLS_EXECUTION__SKILLEXECUTION_HPP