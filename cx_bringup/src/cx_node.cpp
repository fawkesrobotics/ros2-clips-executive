// Copyright (c) 2024 Carologistics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "cx_clips_env_manager/clips_env_manager.hpp"

std::shared_ptr<cx::CLIPSEnvManager> clips_env_manager_node;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe;

void custom_signal_handler(int /*signum*/) {
  if (exe) {
    exe->cancel();
  }
  if (clips_env_manager_node) {
    auto current_state = clips_env_manager_node->get_current_state();
    const std::chrono::seconds timeout_duration(5);

    // Check if the node is active before trying to deactivate
    if (current_state.id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      clips_env_manager_node->trigger_transition(
          lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);

      auto start_time = std::chrono::steady_clock::now();

      while (current_state.id() !=
             lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        current_state = clips_env_manager_node->get_current_state();

        if (std::chrono::steady_clock::now() - start_time >= timeout_duration) {
          RCLCPP_WARN(rclcpp::get_logger("deactivate_and_cleanup_node"),
                      "Timeout waiting for deactivation");
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }

    // After deactivation, check the state
    current_state = clips_env_manager_node->get_current_state();
    // Now trigger cleanup transition if the state is INACTIVE
    if (current_state.id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      clips_env_manager_node->trigger_transition(
          lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);

      auto start_time = std::chrono::steady_clock::now();

      while (current_state.id() !=
             lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
        current_state = clips_env_manager_node->get_current_state();

        if (std::chrono::steady_clock::now() - start_time >= timeout_duration) {
          RCLCPP_WARN(rclcpp::get_logger("deactivate_and_cleanup_node"),
                      "Timeout waiting for cleanup");
          break;
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(10)); // Avoid busy waiting
      }
    }
  }
  if (exe && clips_env_manager_node) {
    exe->remove_node(clips_env_manager_node->get_node_base_interface());
  }
  exe.reset();
  rclcpp::shutdown();
  clips_env_manager_node.reset();
}

int main(int argc, const char **argv) {

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  exe = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  std::signal(SIGINT, custom_signal_handler);

  clips_env_manager_node = std::make_shared<cx::CLIPSEnvManager>();

  exe->add_node(clips_env_manager_node->get_node_base_interface());

  clips_env_manager_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  clips_env_manager_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  exe->spin();
  custom_signal_handler(SIGINT);
  return 0;
}
