#include "ament_index_cpp/get_package_share_directory.hpp"
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <regex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <rclcpp/rclcpp.hpp>

#include "cx_skill_execution/SkillExecution.hpp"
#include "cx_skill_execution/SkillExecutionMaster.hpp"

#include "cx_msgs/msg/skill_action_execinfo.hpp"
#include "cx_msgs/msg/skill_execution.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "gtest/gtest.h"

using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace std::chrono_literals;

class MoveAction : public cx::SkillExecution {
public:
  using Ptr = std::shared_ptr<MoveAction>;
  static Ptr make_shared(const std::string &node_name,
                         const std::string &action,
                         const std::chrono::nanoseconds &rate) {
    return std::make_shared<MoveAction>(node_name, action, rate);
  }

  MoveAction(const std::string &id, const std::string &action_name,
             const std::chrono::nanoseconds &rate)
      : SkillExecution(id, action_name, rate) {
    executions_ = 0;
    cycles_ = 0;
  }

  CallbackReturnT on_activate(const rclcpp_lifecycle::State &state) {
    std::cerr << "MoveAction::on_activate" << std::endl;
    counter_ = 0;

    return SkillExecution::on_activate(state);
  }

  void perform_execution() override {
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

  int counter_;
  int executions_;
  int cycles_;
};

TEST(skill_master_test, skill_master) {
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto test_lf_node =
      rclcpp_lifecycle::LifecycleNode::make_shared("test_lf_node");
  auto move_action_node =
      std::make_shared<MoveAction>("move_action", "move", 1s);
  auto move_action_master = std::make_shared<cx::SkillExecutionMaster>(
      "master_node", "skill-id-1", "move", "",
      "r2d2 steering_wheels_zone assembly_zone");

  for (const auto &param : move_action_master->get_action_params()) {
    RCLCPP_WARN(test_node->get_logger(), "Test param: %s", param.c_str());
  }
  ASSERT_EQ(move_action_master->get_action_name(), "move");
  ASSERT_EQ(move_action_master->get_action_params().size(), 3u);
  ASSERT_EQ(move_action_master->get_action_params()[0], "r2d2");
  ASSERT_EQ(move_action_master->get_action_params()[2], "assembly_zone");

  // move_action_node->set_parameter({"action_name", "move"});

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 8);

  exe.add_node(test_node);
  exe.add_node(test_lf_node->get_node_base_interface());
  exe.add_node(move_action_node->get_node_base_interface());
  exe.add_node(move_action_master->get_node_base_interface());

  std::vector<cx_msgs::msg::SkillExecution> action_execution_msgs;

  auto action_hub_sub =
      test_node->create_subscription<cx_msgs::msg::SkillExecution>(
          "/skill_board", rclcpp::QoS(100).reliable(),
          [&action_execution_msgs](
              const cx_msgs::msg::SkillExecution::SharedPtr msg) {
            RCLCPP_WARN(rclcpp::get_logger("SUB"), "Recieved message!");
            action_execution_msgs.push_back(*msg);
          });

  bool finish = false;
  std::thread t([&]() {
    while (!finish) {
      exe.spin_some();
    }
  });

  ASSERT_EQ(move_action_master->get_exec_status(),
            cx::SkillExecutionMaster::ExecState::IDLE);
  ASSERT_EQ(move_action_master->get_exec_info().status,
            cx_msgs::msg::SkillActionExecinfo::S_IDLE);

  test_lf_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  move_action_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }
  RCLCPP_WARN(test_node->get_logger(), "Out of first loop");

  ASSERT_EQ(move_action_node->get_current_state().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // ASSERT_EQ(move_action_node->get_internal_status().state,
  //           plansys2_msgs::msg::ActionPerformerStatus::READY);
  ASSERT_TRUE(action_execution_msgs.empty());

  {
    move_action_master->request_skill_execution();
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }
  RCLCPP_WARN(test_node->get_logger(), "Out of second loop");

  ASSERT_EQ(move_action_node->get_current_state().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_EQ(move_action_master->get_exec_status(),
            cx::SkillExecutionMaster::ExecState::RUNNING);
  ASSERT_EQ(move_action_master->get_exec_info().status,
            cx_msgs::msg::SkillActionExecinfo::S_RUNNING);

  ASSERT_EQ(action_execution_msgs.size(), 4u);
  ASSERT_EQ(action_execution_msgs[0].type,
            cx_msgs::msg::SkillExecution::REQUEST);
  ASSERT_EQ(action_execution_msgs[1].type,
            cx_msgs::msg::SkillExecution::RESPONSE);
  ASSERT_EQ(action_execution_msgs[2].type,
            cx_msgs::msg::SkillExecution::CONFIRM);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 5) {
      rate.sleep();
    }
  }
  RCLCPP_WARN(test_node->get_logger(), "Out of third loop");

  ASSERT_EQ(move_action_master->get_exec_status(),
            cx::SkillExecutionMaster::ExecState::SUCESS);
  // ASSERT_EQ(move_action_node->get_internal_status().state,
  //           plansys2_msgs::msg::ActionPerformerStatus::READY);

  ASSERT_EQ(action_execution_msgs.size(), 8u);
  ASSERT_EQ(action_execution_msgs[3].type,
            cx_msgs::msg::SkillExecution::FEEDBACK);
  ASSERT_EQ(action_execution_msgs[4].type,
            cx_msgs::msg::SkillExecution::FEEDBACK);
  ASSERT_EQ(action_execution_msgs[5].type,
            cx_msgs::msg::SkillExecution::FEEDBACK);
  ASSERT_EQ(action_execution_msgs[6].type,
            cx_msgs::msg::SkillExecution::FEEDBACK);
  ASSERT_EQ(action_execution_msgs[7].type,
            cx_msgs::msg::SkillExecution::FINISH);

  ASSERT_EQ(move_action_master->get_exec_status(),
            cx::SkillExecutionMaster::ExecState::SUCESS);
  ASSERT_EQ(move_action_master->get_exec_info().status,
            cx_msgs::msg::SkillActionExecinfo::S_FINAL);

  RCLCPP_WARN(test_node->get_logger(), "Hell yeah");

  finish = true;
  t.join();
}
TEST(skill_master_test, skill_master_cancelation) {
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto test_lf_node =
      rclcpp_lifecycle::LifecycleNode::make_shared("test_lf_node");
  auto move_action_node =
      std::make_shared<MoveAction>("move_action", "move", 1s);
  auto move_action_master = std::make_shared<cx::SkillExecutionMaster>(
      "master_node", "skill-id-1", "move", "",
      "r2d2 steering_wheels_zone assembly_zone");

  for (const auto &param : move_action_master->get_action_params()) {
    RCLCPP_WARN(test_node->get_logger(), "Test param: %s", param.c_str());
  }
  ASSERT_EQ(move_action_master->get_action_name(), "move");
  ASSERT_EQ(move_action_master->get_action_params().size(), 3u);
  ASSERT_EQ(move_action_master->get_action_params()[0], "r2d2");
  ASSERT_EQ(move_action_master->get_action_params()[2], "assembly_zone");

  // move_action_node->set_parameter({"action_name", "move"});

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 8);

  exe.add_node(test_node);
  exe.add_node(test_lf_node->get_node_base_interface());
  exe.add_node(move_action_node->get_node_base_interface());
  exe.add_node(move_action_master->get_node_base_interface());

  std::vector<cx_msgs::msg::SkillExecution> action_execution_msgs;

  auto action_hub_sub =
      test_node->create_subscription<cx_msgs::msg::SkillExecution>(
          "/skill_board", rclcpp::QoS(100).reliable(),
          [&action_execution_msgs](
              const cx_msgs::msg::SkillExecution::SharedPtr msg) {
            RCLCPP_WARN(rclcpp::get_logger("SUB"), "Recieved message!");
            action_execution_msgs.push_back(*msg);
          });

  bool finish = false;
  std::thread t([&]() {
    while (!finish) {
      exe.spin_some();
    }
  });

  ASSERT_EQ(move_action_master->get_exec_status(),
            cx::SkillExecutionMaster::ExecState::IDLE);
  ASSERT_EQ(move_action_master->get_exec_info().status,
            cx_msgs::msg::SkillActionExecinfo::S_IDLE);

  test_lf_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  move_action_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }
  RCLCPP_WARN(test_node->get_logger(), "Out of first loop");

  ASSERT_EQ(move_action_node->get_current_state().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // ASSERT_EQ(move_action_node->get_internal_status().state,
  //           plansys2_msgs::msg::ActionPerformerStatus::READY);
  ASSERT_TRUE(action_execution_msgs.empty());

  {
    move_action_master->request_skill_execution();
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }
  RCLCPP_WARN(test_node->get_logger(), "Out of second loop");

  ASSERT_EQ(move_action_node->get_current_state().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_EQ(move_action_master->get_exec_status(),
            cx::SkillExecutionMaster::ExecState::RUNNING);
  ASSERT_EQ(move_action_master->get_exec_info().status,
            cx_msgs::msg::SkillActionExecinfo::S_RUNNING);

  ASSERT_EQ(action_execution_msgs.size(), 4u);
  ASSERT_EQ(action_execution_msgs[0].type,
            cx_msgs::msg::SkillExecution::REQUEST);
  ASSERT_EQ(action_execution_msgs[1].type,
            cx_msgs::msg::SkillExecution::RESPONSE);
  ASSERT_EQ(action_execution_msgs[2].type,
            cx_msgs::msg::SkillExecution::CONFIRM);

  move_action_master->cancel_execution();
  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 5) {
      rate.sleep();
    }
  }
  RCLCPP_WARN(test_node->get_logger(), "Out of third loop");

  ASSERT_EQ(move_action_master->get_exec_status(),
            cx::SkillExecutionMaster::ExecState::CANCELLED);
  // ASSERT_EQ(move_action_node->get_internal_status().state,
  //           plansys2_msgs::msg::ActionPerformerStatus::READY);

  ASSERT_EQ(action_execution_msgs[action_execution_msgs.size() - 2].type,
            cx_msgs::msg::SkillExecution::CANCEL);
  ASSERT_EQ(action_execution_msgs[action_execution_msgs.size() - 1].type,
            cx_msgs::msg::SkillExecution::FINISH);

  ASSERT_EQ(move_action_master->get_exec_status(),
            cx::SkillExecutionMaster::ExecState::CANCELLED);
  ASSERT_EQ(move_action_master->get_exec_info().status,
            cx_msgs::msg::SkillActionExecinfo::S_FAILED);

  RCLCPP_WARN(test_node->get_logger(), "Hell yeah");

  finish = true;
  t.join();
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}