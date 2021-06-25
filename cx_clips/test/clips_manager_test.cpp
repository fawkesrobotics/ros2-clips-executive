#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "gtest/gtest.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "cx_clips/CLIPSEnvManagerClient.hpp"
#include "cx_clips/CLIPSEnvManagerNode.h"

TEST(clips_manager, create_clips_env) {
  auto testing_node = rclcpp::Node::make_shared("testing_node");
  auto manager_node = std::make_shared<cx::CLIPSEnvManagerNode>();
  auto manager_client = std::make_shared<cx::CLIPSEnvManagerClient>();

  const std::string env_name = "executive";
  const std::string log_name = "(clips-executive)";

  std::vector<std::string> features{"f-A", "f-B", "f-C"};
  // std::string pkgPath =
  // ament_index_cpp::get_package_share_directory("cx_clips");

  rclcpp::executors::SingleThreadedExecutor exe;

  exe.add_node(manager_node->get_node_base_interface());
  bool finish_exec = false;
  std::thread t([&]() {
    while (!finish_exec) {
      exe.spin_some();
    }
  });

  using lc_tr = lifecycle_msgs::msg::Transition;

  manager_node->trigger_transition(lc_tr::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start_time = testing_node->now();
    while ((testing_node->now() - start_time).seconds() < 0.5) {
      rate.sleep();
    }
  }

  manager_node->trigger_transition(lc_tr::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start_time = testing_node->now();
    while ((testing_node->now() - start_time).seconds() < 0.5) {
      rate.sleep();
    }
  }

  RCLCPP_INFO(testing_node->get_logger(), "TESTING-NODE-BEGIN");

  ASSERT_TRUE(manager_client->addFeatures(features));

  ASSERT_TRUE(manager_client->createNewClipsEnvironment(env_name, log_name));

  ASSERT_TRUE(manager_client->destroyClipsEnvironment(env_name));

  ASSERT_TRUE(manager_client->assertCanRemoveClipsFeatures(features));

  ASSERT_TRUE(manager_client->removeClipsFeatures(features));

  RCLCPP_INFO(testing_node->get_logger(), "TESTING-NODE-END");

  finish_exec = true;
  t.join();
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
