/***************************************************************************
 *  clips_manager_test.cpp
 *
 *  Created: 25 June 2021
 *  Copyright  2021  Ivaylo Doychev
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

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
  auto manager_client =
      std::make_shared<cx::CLIPSEnvManagerClient>("clips_manager_client");

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
      exe.spin();
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

  // All functions work async!
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
