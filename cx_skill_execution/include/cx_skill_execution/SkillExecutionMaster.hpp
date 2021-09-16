#ifndef CX_SKILL_EXECUTION__SKILLEXECUTIONMASTER_HPP
#define CX_SKILL_EXECUTION__SKILLEXECUTIONMASTER_HPP

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "cx_msgs/msg/skill_action_execinfo.hpp"
#include "cx_msgs/msg/skill_execution.hpp"
#include "cx_msgs/msg/skill_executioner_information.hpp"

namespace cx {

class SkillExecutionMaster : public rclcpp::Node {

public:
  enum ExecState { IDLE, RUNNING, CANCELLED, SUCESS, FAILURE };
  SkillExecutionMaster(
      const std::string &node_name, const std::string &skill_id,
      const std::string &action_name, const std::string &action_parameters,
      const std::string &mapped_action, const std::string &namespace_ = "",
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  void request_skill_execution();
  void cancel_execution();
  ExecState get_exec_status() const;

  cx_msgs::msg::SkillActionExecinfo get_exec_info() const {
    return exec_info_;
  };

  std::string get_action_name() const { return action_name_; }
  std::vector<std::string> get_action_params() const {
    return action_parameters_;
  }

protected:
  void skill_board_cb(const cx_msgs::msg::SkillExecution::SharedPtr msg);
  void confirm_executioner(const std::string &node_id);
  void reject_skill_execution(const std::string &node_id);
  std::vector<std::string> extract_parameters(const std::string &action_params);

protected:
  const std::string skill_id_;
  const std::string action_name_;
  const std::string action_mapping_;
  const std::string mapped_action_;
  const std::string string_action_parameters_;
  std::vector<std::string> action_parameters_;
  std::string executioner_id_;

  ExecState state_;
  cx_msgs::msg::SkillActionExecinfo exec_info_{};
  std::string feedback_{};

  rclcpp::Time current_state_time_;
  rclcpp::Time exec_start_;

  rclcpp::Publisher<cx_msgs::msg::SkillExecution>::SharedPtr skill_board_pub;
  rclcpp::Subscription<cx_msgs::msg::SkillExecution>::SharedPtr skill_board_sub;
};
} // namespace cx
#endif // !CX_SKILL_EXECUTION__SKILLEXECUTIONMASTER_HPP