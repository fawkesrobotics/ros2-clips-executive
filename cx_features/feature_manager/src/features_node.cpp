// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "cx_feature_manager/clips_feature_manager.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cx::ClipsFeatureManager>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
