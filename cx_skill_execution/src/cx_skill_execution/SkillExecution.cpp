/*
  Classes implmenting a ceratin action should
  inherit from the class and implement their
  individual behaviour.
*/
#include <memory>
#include <string>

#include "cx_skill_execution/SkillExecution.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace cx {
using namespace std::chrono_literals;
using SkillExecutionerInformation = cx_msgs::msg::SkillExecutionerInformation;
using SkillExecutionMsg = cx_msgs::msg::SkillExecution;

SkillExecution::SkillExecution(const std::string &node_name,
                               const std::string &action_name,
                               const std::chrono::nanoseconds &exec_rate,
                               const rclcpp::NodeOptions &options,
                               const std::string &namespace_)
    : rclcpp_lifecycle::LifecycleNode(node_name, namespace_, options),
      action_name_(action_name), exec_rate_(exec_rate) {
  RCLCPP_INFO(get_logger(), "Initialising executioner %s for skill %s",
              node_name.c_str(), action_name.c_str());
  executioner_info_.state = SkillExecutionerInformation::INIT;
  executioner_info_.action_name = action_name;
  executioner_info_.node_name = get_name();
}

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using std::placeholders::_1;

CallbackReturn
SkillExecution::on_configure(const rclcpp_lifecycle::State &state) {
  executioner_info_pub_ = create_publisher<SkillExecutionerInformation>(
      "/skill_executioner_status", rclcpp::QoS(100).reliable());
  executioner_info_pub_->on_activate();
  // Publish the current executioner information
  executioner_info_heartbeat_ = create_wall_timer(
      1s, [this]() { executioner_info_pub_->publish(executioner_info_); });

  skill_board_pub = create_publisher<SkillExecutionMsg>(
      "/skill_board", rclcpp::QoS(100).reliable());
  skill_board_sub = create_subscription<SkillExecutionMsg>(
      "/skill_board", rclcpp::QoS(100).reliable(),
      std::bind(&SkillExecution::skill_board_cb, this, _1));

  skill_board_pub->on_activate();
  executioner_info_.state = SkillExecutionerInformation::READY;
  return CallbackReturn::SUCCESS;
}

CallbackReturn
SkillExecution::on_activate(const rclcpp_lifecycle::State &state) {
  executioner_info_.state = SkillExecutionerInformation::RUNNING;
  execution_heartbeat_ = create_wall_timer(
      exec_rate_, std::bind(&SkillExecution::perform_execution, this));
  perform_execution();
  return CallbackReturn::SUCCESS;
}
CallbackReturn
SkillExecution::on_deactivate(const rclcpp_lifecycle::State &state) {
  executioner_info_.state = SkillExecutionerInformation::READY;
  executioner_info_pub_->on_deactivate();
  execution_heartbeat_ = nullptr;
  return CallbackReturn::SUCCESS;
}

void SkillExecution::skill_board_cb(
    const cx_msgs::msg::SkillExecution::SharedPtr msg) {
  // Check the type of the published message
  /*
    Important types for the executioner:
    1. Request - coming from clips feature/clips directly
    2. Confirm - give confirmation that the skill will be executed
    3. Reject - request to stop skill execution
    4. Cancel - cancel skill execution
  */
  switch (msg->type) {
  case SkillExecutionMsg::RESPONSE:
  case SkillExecutionMsg::FEEDBACK:
  case SkillExecutionMsg::FINISH:
    break;
  case SkillExecutionMsg::REQUEST:
    if (get_current_state().id() ==
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE &&
        !commited_to_skill_ && action_name_ == msg->action) {
      commited_to_skill_ = true;
      send_response(msg);
    }
    break;
  case SkillExecutionMsg::CONFIRM:
    if (get_current_state().id() ==
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE &&
        msg->node_id == get_name() && commited_to_skill_) {
      action_parameters_ = msg->action_parameters;
      mapped_action_ = msg->mapped_action;
      // Transition to active state, so the implemented function can be executed
      trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    }
    break;
  case SkillExecutionMsg::REJECT:
    if (msg->node_id == get_name()) {
      commited_to_skill_ = false;
    }
    break;
  case SkillExecutionMsg::CANCEL:
    if (get_current_state().id() ==
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE &&
        msg->node_id == get_name()) {
      finish_execution(false, 0.0,
                       /*expected from the exec master*/ "CANCELLED");
    }
    break;
  default:
    RCLCPP_ERROR(get_logger(), "Message type: %d not recognized for action %s!",
                 msg->type, action_name_.c_str());
    break;
  }
}

void SkillExecution::send_response(
    const cx_msgs::msg::SkillExecution::SharedPtr msg) {
  SkillExecutionMsg rsp = *msg;
  rsp.type = SkillExecutionMsg::RESPONSE;
  rsp.node_id = get_name();
  skill_board_pub->publish(rsp);
}

void SkillExecution::send_feedback(float progress, const std::string &status) {
  SkillExecutionMsg msg;
  msg.type = SkillExecutionMsg::FEEDBACK;
  msg.node_id = get_name();
  msg.action = action_name_;
  msg.action_parameters = action_parameters_;
  msg.mapped_action + mapped_action_;
  msg.progress = progress;
  msg.status = status;

  skill_board_pub->publish(msg);
}

void SkillExecution::finish_execution(bool success, float progress,
                                      const std::string &status) {
  if (get_current_state().id() ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  }

  SkillExecutionMsg msg;
  msg.type = SkillExecutionMsg::FINISH;
  msg.node_id = get_name();
  msg.action = action_name_;
  msg.action_parameters = action_parameters_;
  msg.mapped_action + mapped_action_;
  msg.progress = progress;
  msg.status = status;
  msg.success = success;

  skill_board_pub->publish(msg);
}

} // namespace cx
