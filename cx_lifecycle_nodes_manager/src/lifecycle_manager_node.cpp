// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "cx_lifecycle_nodes_manager/LifecycleNodesManager.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  auto node = std::make_shared<cx::LifecycleNodesManager>();
  exe.add_node(node);
  // Calls the init function of the Manager
  auto start_time = node->now();
  auto req_time = (node->now() - start_time).seconds();
  while (1) {
    req_time = (node->now() - start_time).seconds();
    if (req_time > 0.75) {
      break;
    }
  }
  node->initialise();
  exe.spin_some();
  // rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
