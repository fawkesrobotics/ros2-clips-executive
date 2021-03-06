#include <memory>
#include <string>

#include "cx_clips/CLIPSEnvManagerNode.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, const char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cx::CLIPSEnvManagerNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
