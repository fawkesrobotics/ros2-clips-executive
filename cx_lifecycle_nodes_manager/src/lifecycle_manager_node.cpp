#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "cx_lifecycle_nodes_manager/LifecycleNodesManager.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cx::LifecycleNodesManager>();
  // Calls the init function of the Manager
  node->initialise();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
