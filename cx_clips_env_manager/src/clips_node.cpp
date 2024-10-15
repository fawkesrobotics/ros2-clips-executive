// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#include <memory>
#include <string>

#include "cx_clips_env_manager/clips_env_manager.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, const char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cx::CLIPSEnvManager>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
