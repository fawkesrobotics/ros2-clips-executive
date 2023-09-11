#include <chrono>
#include <memory>
#include <string>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "cx_example_skill_nodes/MoveSkillNav2.hpp"
#include "cx_example_skill_nodes/SimulatorSkill.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // navigate skill node
  auto navigate_node = std::make_shared<cx::SimulatorSkill>(
      "simulator_skill_node", "navigate", 500ms);

  std::thread navigate_node_t([&navigate_node]() {
    navigate_node->set_parameter(rclcpp::Parameter("agent_id", ""));
    navigate_node->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    rclcpp::spin(navigate_node->get_node_base_interface());
  });

  // soil sampling skill node
  auto soil_node = std::make_shared<cx::SimulatorSkill>("simulator_skill_node",
                                                        "sample_soil", 500ms);

  std::thread soil_node_t([&soil_node]() {
    soil_node->set_parameter(rclcpp::Parameter("agent_id", ""));
    soil_node->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    rclcpp::spin(soil_node->get_node_base_interface());
  });

  // rock sampling skill node
  auto rock_node = std::make_shared<cx::SimulatorSkill>("simulator_skill_node",
                                                        "sample_rock", 500ms);

  std::thread rock_node_t([&rock_node]() {
    rock_node->set_parameter(rclcpp::Parameter("agent_id", ""));
    rock_node->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    rclcpp::spin(rock_node->get_node_base_interface());
  });

  // drop sampling skill node
  auto drop_node = std::make_shared<cx::SimulatorSkill>("simulator_skill_node",
                                                        "drop", 500ms);
  std::thread drop_node_t([&drop_node]() {
    drop_node->set_parameter(rclcpp::Parameter("agent_id", ""));
    drop_node->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    rclcpp::spin(drop_node->get_node_base_interface());
  });

  navigate_node_t.join();
  drop_node_t.join();
  soil_node_t.join();
  rock_node_t.join();

  rclcpp::shutdown();
  return 0;
}
