// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#include <chrono>
#include <memory>
#include <string>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "cx_example_skill_nodes/dummy_skill.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cx::DummySkill>("dummy_skill_node", 500ms);

  node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
