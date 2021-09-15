#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "cx_clips/CLIPSEnvManagerNode.h"
#include "cx_clips_executive/ClipsExecutive.hpp"
#include "cx_features/ClipsFeaturesManager.hpp"

int main(int argc, const char **argv) {

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  auto clips_env_manager_node = std::make_shared<cx::CLIPSEnvManagerNode>();
  auto clips_features_manager_node =
      std::make_shared<cx::ClipsFeaturesManager>();
  auto clips_executive_node = std::make_shared<cx::ClipsExecutive>();

  exe.add_node(clips_env_manager_node->get_node_base_interface());
  exe.add_node(clips_features_manager_node->get_node_base_interface());
  exe.add_node(clips_executive_node->get_node_base_interface());

  clips_features_manager_node->pre_configure(clips_env_manager_node);
  clips_executive_node->pre_configure(clips_env_manager_node);

  clips_env_manager_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  clips_env_manager_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  RCLCPP_INFO(rclcpp::get_logger("CX_NODE"), "Triggered transition!");

  exe.spin();

  rclcpp::shutdown();

  return 0;
}